#!/usr/bin/env python

import argparse
import sys
import rclpy
from std_msgs.msg import Float64, Bool
from mrobosub_lib import Node

from typing import Optional, Final


class PassthroughDofController(Node):
    """
    Subscribers
    - /target_twist/surge (power)

    Publishers
    - /output_wrench/surge
    """

    def __init__(self, dof_name: str):
        super().__init__(f"{dof_name}_control")
        self.output_pub = self.create_publisher(
            Float64, f"/output_wrench/{dof_name}", qos_profile=1
        )
        self.create_subscription(
            Float64,
            f"/target_twist/{dof_name}",
            self.target_twist_callback,
            qos_profile=1,
        )

    def target_twist_callback(self, target_twist: Float64):
        self.pub_output_dof(target_twist.data)

    def pub_output_dof(self, output: float):
        self.output_pub.publish(output)

    def cleanup(self):
        self.output_pub.publish(0)


def main():
    rclpy.init()
    
    parser = argparse.ArgumentParser()
    parser.add_argument("dof_name", type=str, help="Name of the DOF")
    args = parser.parse_args(sys.argv[1:2])

    node = PassthroughDofController(args.dof_name)

    rclpy.spin(node)

    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()
