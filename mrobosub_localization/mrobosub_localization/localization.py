import math
import numpy as np
import rclpy
from std_msgs.msg import Float64, Float32

from mrobosub_lib import Node

from typing import Optional, Final

from std_srvs.srv import Trigger
from geometry_msgs.msg import Quaternion
from mrobosub_msgs.msg import ImuINS

from math import degrees


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0.0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


class StateEstimation(Node):
    """
    Subscribers
    - /depth/raw_depth
    - /imu_INS
    """

    """
    Publishers
    - /pose/heave
    - /pose/yaw
    - /pose/pitch
    - /pose/roll
    """
    # pid_params: PIDParams

    heave_offset = None
    yaw_offset = None
    pitch_offset = None
    roll_offset = None

    orientation = None

    def __init__(self):
        super().__init__("localization")
        self.heave_pub = self.create_publisher(Float64, "/pose/heave", qos_profile=1)
        self.yaw_pub = self.create_publisher(Float64, "/pose/yaw", qos_profile=1)
        self.pitch_pub = self.create_publisher(Float64, "/pose/pitch", qos_profile=1)
        self.roll_pub = self.create_publisher(Float64, "/pose/roll", qos_profile=1)
        self.create_subscription(
            Float32, "/depth/raw_depth", self.raw_depth_callback, qos_profile=1
        )
        self.create_subscription(ImuINS, "/imu_INS", self.imu_callback, qos_profile=1)
        self.create_service(Trigger, "localization/zero_state", self.handle_reset)

    def handle_reset(
        self, req: Trigger.Request, res: Trigger.Response
    ) -> Trigger.Response:
        previous_offsets = f"{self.heave_offset=}, {self.yaw_offset=}, {self.pitch_offset=}, {self.roll_offset=}"

        self.heave_offset = None
        self.yaw_offset = None
        self.pitch_offset = None
        self.roll_offset = None

        self.heave_pub.publish(Float64(data=0))
        self.yaw_pub.publish(Float64(data=0))
        self.pitch_pub.publish(Float64(data=0))
        self.roll_pub.publish(Float64(data=0))

        res.success = True
        res.message = previous_offsets

        return res

    def raw_depth_callback(self, raw_depth: Float32):
        if self.heave_offset is None:
            self.heave_offset = raw_depth.data
        self.heave_pub.publish(Float32(data=raw_depth.data - self.heave_offset))

    def imu_callback(self, msg: ImuINS):

        # orientation = msg.orientation
        #
        # quaternion = [
        #     orientation.x,
        #     orientation.y,
        #     orientation.z,
        #     orientation.w
        # ]
        # euler = euler_from_quaternion(quaternion)
        euler = msg.theta

        if (
            self.yaw_offset is None
            or self.pitch_offset is None
            or self.roll_offset is None
        ):
            self.yaw_offset = degrees(-euler.z)
            self.pitch_offset = degrees(-euler.y)
            self.roll_offset = degrees(euler.x)

        yaw = degrees(-euler.z) - self.yaw_offset
        pitch = degrees(-euler.y) - self.pitch_offset
        roll = degrees(euler.x) - self.roll_offset

        self.yaw_pub.publish(Float32(data=yaw))
        self.pitch_pub.publish(Float32(data=pitch))
        self.roll_pub.publish(Float32(data=roll))


def main(args=None):
    rclpy.init(args=args)

    node = StateEstimation()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
