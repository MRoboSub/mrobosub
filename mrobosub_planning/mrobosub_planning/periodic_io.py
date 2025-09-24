import math
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

# from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse, PathmarkerAngle  # type: ignore
from enum import Enum, auto
from std_srvs.srv import SetBool
from dataclasses import dataclass


def angle_error(setpoint: float, state: float) -> float:
    return (((setpoint - state) % 360) + 360) % 360


class ImageTarget(Enum):
    GATE_BLUE = auto()
    GATE_RED = auto()


@dataclass
class Pose:
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    x: float = 0.0
    y: float = 0.0
    heave: float = 0.0


class Interface:
    """
    Public interface class for publishers and subscribers
    """

    def __init__(self, node: Node):
        self.logger = node.get_logger()
        self.node = node

        # Poses
        self.pose = Pose()
        self.target_pose = Pose()

        # Subscribers
        self._yaw_sub = node.create_subscription(
            Float64, "/pose/yaw", self.yaw_callback, 10
        )
        self._pitch_sub = node.create_subscription(
            Float64, "/pose/pitch", self.pitch_callback, 10
        )
        self._roll_sub = node.create_subscription(
            Float64, "/pose/roll", self.roll_callback, 10
        )
        self._x_sub = node.create_subscription(Float64, "/pose/x", self.x_callback, 10)
        self._y_sub = node.create_subscription(Float64, "/pose/y", self.y_callback, 10)
        self._heave_sub = node.create_subscription(
            Float64, "/pose/heave", self.heave_callback, 10
        )

        # Publishers
        self._target_pose_yaw_pub = node.create_publisher(
            Float64, "/target_pose/yaw", 1
        )
        self._target_pose_pitch_pub = node.create_publisher(
            Float64, "/target_pose/pitch", 1
        )
        self._target_pose_roll_pub = node.create_publisher(
            Float64, "/target_pose/roll", 1
        )
        self._target_pose_x_pub = node.create_publisher(Float64, "/target_pose/x", 1)
        self._target_pose_y_pub = node.create_publisher(Float64, "/target_pose/y", 1)
        self._target_pose_heave_pub = node.create_publisher(
            Float64, "/target_pose/heave", 1
        )

        self._target_twist_yaw_pub = node.create_publisher(
            Float64, "/target_twist/yaw", 1
        )
        self._target_twist_pitch_pub = node.create_publisher(
            Float64, "/target_twist/pitch", 1
        )
        self._target_twist_roll_pub = node.create_publisher(
            Float64, "/target_twist/roll", 1
        )
        self._target_twist_surge_pub = node.create_publisher(
            Float64, "/target_twist/surge", 1
        )
        self._target_twist_sway_pub = node.create_publisher(
            Float64, "/target_twist/sway", 1
        )
        self._target_twist_heave_pub = node.create_publisher(
            Float64, "/target_twist/heave", 1
        )

        self._left_dropper_pub = node.create_publisher(Int32, "/left_servo/angle", 1)
        self._right_dropper_pub = node.create_publisher(Int32, "/right_servo/angle", 1)

        # Services
        # TODO: Add services for perception topics when those are created.

        self._zed_on_srv = node.create_client(SetBool, "/zed/on")
        attempt_counter = 0
        while (
            not self._zed_on_srv.wait_for_service(timeout_sec=1.0)
            and attempt_counter < 5
        ):
            self.logger.info('"/zed/on" service not available, waiting again...')
            attempt_counter += 1
        if attempt_counter == 5:
            self.logger.error('Failed to connect to "/zed/on" service')

        attempt_counter = 0
        self._bot_cam_on_srv = node.create_client(SetBool, "/bot_cam/on")
        while (
            not self._bot_cam_on_srv.wait_for_service(timeout_sec=1.0)
            and attempt_counter < 5
        ):
            self.logger.info('"/bot_cam/on" service not available, waiting again...')
            attempt_counter += 1
        if attempt_counter == 5:
            self.logger.error('Failed to connect to "/bot_cam/on" service')

    def is_yaw_within_threshold(self, threshold: float) -> float:
        return abs(angle_error(self.target_pose.yaw, self.pose.yaw)) <= threshold

    def is_magnitude_within_threshold(self, threshold: float) -> float:
        magnitude: float = self.calculate_distance_to_target()
        return magnitude <= threshold

    def is_heave_within_threshold(self, threshold: float) -> float:
        return abs(self.target_pose.heave - self.pose.heave) <= threshold

    def calculate_yaw_to_target(self) -> float:
        # the direction vector to the target d = v2 - v1
        # the angle to this would be arctan(dy / dx)
        dx = self.target_pose.x - self.pose.x
        dy = self.target_pose.y - self.pose.y

        return math.atan2(dy, dx) * 180 / math.pi

    def calculate_distance_to_target(self) -> float:
        dx = self.target_pose.x - self.pose.x
        dy = self.target_pose.y - self.pose.y
        return math.sqrt(dx**2 + dy**2)

    def set_target_pose_yaw(self, target_yaw: float) -> None:
        msg = Float64()
        msg.data = float(target_yaw)
        self._target_pose_yaw_pub.publish(msg)
        self.target_pose.yaw = target_yaw

    def set_target_pose_pitch(self, target_pitch: float) -> None:
        msg = Float64()
        msg.data = float(target_pitch)
        self._target_pose_pitch_pub.publish(msg)
        self.target_pose.pitch = target_pitch

    def set_target_pose_roll(self, target_roll: float) -> None:
        msg = Float64()
        msg.data = float(target_roll)
        self._target_pose_roll_pub.publish(msg)
        self.target_pose.roll = target_roll

    def set_target_pose_x(self, target_x: float) -> None:
        msg = Float64()
        msg.data = float(target_x)
        self._target_pose_x_pub.publish(msg)
        self.target_pose.x = target_x

    def set_target_pose_y(self, target_y: float) -> None:
        msg = Float64()
        msg.data = float(target_y)
        self._target_pose_y_pub.publish(msg)
        self.target_pose.y = target_y

    def set_target_pose_heave(self, target_heave: float) -> None:
        msg = Float64()
        msg.data = float(target_heave)
        self._target_pose_heave_pub.publish(msg)
        self.target_pose.heave = target_heave

    def set_target_twist_yaw(self, override_yaw: float) -> None:
        msg = Float64()
        msg.data = float(override_yaw)
        self._target_twist_yaw_pub.publish(msg)

    def set_target_twist_pitch(self, override_pitch: float) -> None:
        msg = Float64()
        msg.data = float(override_pitch)
        self._target_twist_pitch_pub.publish(msg)

    def set_target_twist_roll(self, override_roll: float) -> None:
        msg = Float64()
        msg.data = float(override_roll)
        self._target_twist_roll_pub.publish(msg)

    def set_target_twist_surge(self, override_surge: float) -> None:
        msg = Float64()
        msg.data = float(override_surge)
        self._target_twist_surge_pub.publish(msg)

    def set_target_twist_sway(self, override_sway: float) -> None:
        msg = Float64()
        msg.data = float(override_sway)
        self._target_twist_sway_pub.publish(msg)

    def set_target_twist_heave(self, override_heave: float) -> None:
        msg = Float64()
        msg.data = float(override_heave)
        self._target_twist_heave_pub.publish(msg)

    def reset_target_twist(self) -> None:
        self.set_target_twist_yaw(0.0)
        self.set_target_twist_pitch(0.0)
        self.set_target_twist_roll(0.0)
        self.set_target_twist_surge(0.0)
        self.set_target_twist_sway(0.0)
        self.set_target_twist_heave(0.0)

    def set_left_dropper_angle(self, angle: int) -> None:
        msg = Int32()
        msg.data = int(angle)
        self._left_dropper_pub.publish(msg)

    def set_right_dropper_angle(self, angle: int) -> None:
        msg = Int32()
        msg.data = int(angle)
        self._right_dropper_pub.publish(msg)

    def activate_zed(self) -> bool:
        bot_cam_req = self._bot_cam_on_srv.Request(data=False)
        bot_cam_res = self._bot_cam_on_srv.call(bot_cam_req, timeout=2.0)

        zed_req = self._zed_on_srv.Request(data=True)
        zed_res = self._zed_on_srv.call(zed_req, timeout=2.0)

        success = bot_cam_res.success and zed_res.success
        return success

    def activate_bot_cam(self) -> bool:
        zed_req = self._zed_on_srv.Request(data=False)
        zed_res = self._zed_on_srv.call(zed_req, timeout=2.0)

        bot_cam_req = self._bot_cam_on_srv.Request(data=True)
        bot_cam_res = self._bot_cam_on_srv.call(bot_cam_req, timeout=2.0)

        success = bot_cam_res.success and zed_res.success
        return success

    def deactivate_cameras(self) -> bool:
        zed_req = self._zed_on_srv.Request(data=False)
        zed_res = self._zed_on_srv.call(zed_req, timeout=2.0)

        bot_cam_req = self._bot_cam_on_srv.Request(data=False)
        bot_cam_res = self._bot_cam_on_srv.call(bot_cam_req, timeout=2.0)

        success = bot_cam_res.success and zed_res.success
        return success

    def yaw_callback(self, msg: Float64) -> None:
        self.pose.yaw = msg.data

    def pitch_callback(self, msg: Float64) -> None:
        self.pose.pitch = msg.data

    def roll_callback(self, msg: Float64) -> None:
        self.pose.roll = msg.data

    def x_callback(self, msg: Float64) -> None:
        self.pose.x = msg.data

    def y_callback(self, msg: Float64) -> None:
        self.pose.y = msg.data

    def heave_callback(self, msg: Float64) -> None:
        self.pose.heave = msg.data
