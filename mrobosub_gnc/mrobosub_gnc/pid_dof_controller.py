#!/usr/bin/env python

import rclpy
import sys
import argparse

from mrobosub_lib import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64
from .pid_interface import PIDInterface

from typing import Optional, Final


class PidDofControlNode(Node):
    """
    Subscribers
    - /target_pose/{dof_name} (deg)
    - /pose/{dof_name} (deg)
    - /target_twist/{dof_name} (power)
    """

    """
    Publishers
    - /output_wrench/{dof_name}
    """
    # pid_params: PIDParams

    output = 0

    def __init__(self, dof_name: str):
        super().__init__(f"{dof_name}_control")
        self.get_logger().info(f"Starting Controller Node for: {dof_name}")

        self.declare_parameter(
            "feed_forward",
            rclpy.Parameter.Type.DOUBLE,
            descriptor=ParameterDescriptor(description="Feed forward control value"),
        )
        self.declare_parameter(
            "max_output",
            rclpy.Parameter.Type.DOUBLE,
            descriptor=ParameterDescriptor(description="Maximum output value"),
        )
        self.declare_parameter(
            "clamp",
            rclpy.Parameter.Type.BOOL,
            descriptor=ParameterDescriptor(description="Clamp output to max_output"),
        )

        self.pid = PIDInterface(self, f"{dof_name}_pid", self.pid_callback)
        self.output_pub = self.create_publisher(
            Float64, f"/output_wrench/{dof_name}", qos_profile=10
        )
        self.create_subscription(
            Float64,
            f"/target_pose/{dof_name}",
            self.target_pose_callback,
            qos_profile=1,
        )
        self.create_subscription(
            Float64, f"/pose/{dof_name}", self.pose_callback, qos_profile=1
        )
        self.create_subscription(
            Float64, f"/target_twist/{dof_name}", self.target_twist, qos_profile=1
        )

    def target_pose_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)

    def pose_callback(self, pose: Float64):
        self.output = pose.data
        self.pid.set_current(pose.data)

    def target_twist(self, target_twist: Float64):
        self.pid.disable()
        self.pub_output(target_twist.data)

    def pid_callback(self, effort: float):
        output = (
            effort
            + self.get_parameter("feed_forward").get_parameter_value().double_value
        )
        self.pub_output(output)

    def pub_output(self, output: float):
        # Protect against running to ground
        if (
            self.output
            > self.get_parameter("max_output").get_parameter_value().double_value
            and self.get_parameter("clamp").get_parameter_value().bool_value
        ):
            output = min(0, output)
        self.output_pub.publish(Float64(data=output))

    def cleanup(self):
        self.output_pub.publish(Float64(data=0.0))


def main():
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("dof_name", type=str, help="Name of the DOF")
    args = parser.parse_args(sys.argv[1:2])

    node = PidDofControlNode(args.dof_name)

    rclpy.spin(node)

    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
