import math
from typing_extensions import NamedTuple
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from std_msgs.msg import Float64, Bool, Int32
# from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse, PathmarkerAngle  # type: ignore
from typing import Dict, Type, Mapping, Optional, Tuple
from enum import Enum, auto
from std_srvs.srv import SetBool
from dataclasses import dataclass


def angle_error(setpoint: float, state: float) -> float:
    return (((setpoint - state) % 360) + 360) % 360


Namespace = Type


class ImageTarget(Enum):
    GATE_BLUE = auto()
    GATE_RED = auto()


# ImageDetections = Dict[ImageTarget, ObjectPositionResponse]

@dataclass
class Pose:
    yaw:float = 0.0
    heave:float = 0.0
    roll:float = 0.0
    x:float = 0.0
    y:float = 0.0

class Captain(Node):
    '''
    Public interface class for publishers and subscribers
    '''
    def __init__(self, name:str='captain'):
        '''
        @param name - name of the node (should be captain)
        '''
        super().__init__(name)

        #Poses
        self.pose = Pose()
        self.target_pose = Pose()

        # Subscribers
        self._yaw_sub = self.create_subscription(Float64, "/pose/yaw", self.yaw_callback, 10)
        self._heave_sub = self.create_subscription(Float64, "/pose/heave", self.heave_callback, 10)
        self._roll_sub = self.create_subscription(Float64, "/pose/roll", self.roll_callback, 10)
        self._x_sub = self.create_subscription(Float64, "/pose/x", self.x_callback, 10)
        self._y_sub = self.create_subscription(Float64, "/pose/y", self.y_callback, 10)

        # Publishers
        self._target_pose_heave_pub = self.create_publisher(Float64, "/target_pose/heave", 1)
        self._target_pose_yaw_pub = self.create_publisher(Float64, "/target_pose/yaw", 1)
        self._target_pose_roll_pub = self.create_publisher(Float64, "/target_pose/roll", 1)
        self._target_pose_x_pub = self.create_publisher(Float64, "/target_pose/x", 1)
        self._target_pose_y_pub = self.create_publisher(Float64, "/target_pose/y", 1)

        self._target_twist_yaw_pub = self.create_publisher(Float64, "/target_twist/yaw", 1)
        self._target_twist_roll_pub = self.create_publisher(Float64, "/target_twist/roll", 1)
        self._target_twist_surge_pub = self.create_publisher(Float64, "/target_twist/surge", 1)
        self._target_twist_sway_pub = self.create_publisher(Float64, "/target_twist/sway", 1)
        self._target_twist_heave_pub = self.create_publisher(Float64, "/target_twist/heave", 1)

        self._left_dropper_pub = self.create_publisher(Int32, "/left_servo/angle", 1)
        self._right_dropper_pub = self.create_publisher(Int32, "/right_servo/angle", 1)

        # Services
        # TODO: Don't need these until perception is properly updated / new game states
        # _pathmarker_srv = rospy.ServiceProxy(
        #     "/pathmarker_angle", PathmarkerAngle, persistent=True
        # )
        # _bin_cam_pos_srv = rospy.ServiceProxy(
        #     "/bin_object_position", ObjectPosition, persistent=True
        # )
        # _hsv_buoy_position_srv = rospy.ServiceProxy(
        #     "/buoy_object_position", ObjectPosition, persistent=True
        # )

        self._zed_on_srv = self.create_client(SetBool, "/zed/on")
        attempt_counter = 0
        while not self._zed_on_srv.wait_for_service(timeout_sec=1.0) and attempt_counter < 5:
            self.get_logger().info('\"/zed/on\" service not available, waiting again...')
            attempt_counter += 1
        if attempt_counter == 5:
            self.get_logger().error('Failed to connect to \"/zed/on\" service')

        attempt_counter = 0
        self._bot_cam_on_srv = self.create_client(SetBool, "/bot_cam/on")
        while not self._bot_cam_on_srv.wait_for_service(timeout_sec=1.0) and attempt_counter < 5:
            self.get_logger().info('\"/bot_cam/on\" service not available, waiting again...')
            attempt_counter += 1
        if attempt_counter == 5:
            self.get_logger().error('Failed to connect to "/bot_cam/on\" service')

    # def query_BinCamPos(cls) -> Optional[ObjectPositionResponse]:
    #     """Request the x, y position on the camera of the bin (0,0) being the center +y is up and +x is right,
    #       and found which is True if we have data

    #     Returns:
    #         x, y position on the camera of the bin and found
    #         None otherwise.
    #     """

    #     try:
    #         resp = cls._bin_cam_pos_srv()
    #     except rospy.service.ServiceException as e:
    #         print("Cannot reach bin object position service")
    #         return None
    #     if resp.found:
    #         return resp
    #     else:
    #         return None

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

    def set_target_pose_heave(self, target_heave: float) -> None:
        msg = Float64()
        msg.data = float(target_heave)
        self._target_pose_heave_pub.publish(msg)
        self.target_pose.heave = target_heave

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

    def set_target_twist_roll(self, override_roll: float) -> None:
        msg = Float64()
        msg.data = float(override_roll)
        self._target_twist_roll_pub.publish(msg)

    def set_target_twist_yaw(self, override_yaw: float) -> None:
        msg = Float64()
        msg.data = float(override_yaw)
        self._target_twist_yaw_pub.publish(msg)

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
        self.set_target_twist_heave(0.)
        self.set_target_twist_yaw(0.)
        self.set_target_twist_surge(0.)
        self.set_target_twist_roll(0.)
        self.set_target_twist_sway(0.)

    def set_left_dropper_angle(self, angle: int) -> None:
        msg = Int32()
        msg.data = int(angle)
        self._left_dropper_pub.publish(msg)

    def set_right_dropper_angle(self, angle: int) -> None:
        msg = Int32()
        msg.data = int(angle)
        self._right_dropper_pub.publish(msg)

    # @classmethod
    # def query_pathmarker(cls) -> Optional[float]:
    #     """Request a the pathmaker angle.

    #     Returns:
    #         angle of path marker in global frame (i.e. same frame as the Pose.yaw) if found
    #         None otherwise.
    #     """

    #     pm_pos = cls.query_pathmarker_full()
    #     if pm_pos is None:
    #         return None
    #     return pm_pos.angle

    # class PathmarkerPosition(NamedTuple):
    #     centroid_x: float
    #     centroid_y: float
    #     angle: float

    # @classmethod
    # def query_pathmarker_full(cls) -> Optional[PathmarkerPosition]:
    #     try:
    #         resp = cls._pathmarker_srv()
    #     except rospy.service.ServiceException as e:
    #         print("Pathmarker service is not active")
    #         return None
    #     print(f"{resp=}")
    #     if not resp.found:
    #         return None
    #     convertedAngle: float = (90 + resp.angle) + cls.Pose.yaw
    #     if convertedAngle > 90:  # if pointing behind us flip 180
    #         convertedAngle -= 180
    #     elif convertedAngle < -90:
    #         convertedAngle += 180
    #     return cls.PathmarkerPosition(
    #         centroid_x=resp.centroid_x,
    #         centroid_y=resp.centroid_y,
    #         angle=convertedAngle,
    #     )

    # @classmethod
    # def query_buoy(cls) -> ObjectPositionResponse:
    #     try:
    #         return cls._hsv_buoy_position_srv()
    #     except rospy.ServiceException as e:
    #         print(f"Buoy service cannot be called with error: {e}")
    #         obj_msg = ObjectPositionResponse()
    #         obj_msg.found = False
    #         return obj_msg

    # @classmethod
    # def query_image(cls, image_type: Optional[ImageTarget]) -> ObjectPositionResponse:
    #     if image_type is not None:
    #         try:
    #             return cls._object_position_srvs[image_type]()
    #         except rospy.ServiceException:
    #             pass
    #     obj_msg = ObjectPositionResponse()
    #     obj_msg.found = False
    #     return obj_msg

    # @classmethod
    # def query_all_images(cls) -> ImageDetections:
    #     results = {}
    #     for g in ImageTarget:
    #         resp = cls.query_image(g)
    #         if resp.found:
    #             results[g] = resp
    #     return results

    def _call_service(self, service: Service, request: SetBool.Request, error_string:str) -> bool:
        success = True
        future = service.call_async(request)
        #timeout is set to 2s 
        # Potentially can add future callbacks to perform this async, but for now like this
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            self.get_logger().error(f"{error_string}: Service call timed out")
            success = False
        exc = future.execption()
        if success and exc:
            self.get_logger().error(f"{error_string}: {exc!r}")
            success = False
        return success



    def activate_zed(self) -> bool:
        self.req = self._bot_cam_on_srv.Request()
        self.req.data = False

        success = self._call_service(self._bot_cam_on_srv, self.req, "Turning BotCam Off")

        self.req = self._zed_on_srv.Request()
        self.req.data = True

        success = success and self._call_service(self._zed_on_srv, self.req, "Turning Zed On")
        return success

    def activate_bot_cam(self) -> bool:
        self.req = self._zed_on_srv.Request()
        self.req.data = False

        success = self._call_service(self._zed_on_srv, self.req, "Turning Zed Off")

        self.req = self._bot_cam_on_srv.Request()
        self.req.data = True

        success = success and self._call_service(self._bot_cam_on_srv, self.req, "Turning BotCam On")
        return success

    def deactivate_cameras(self) -> bool:
        self.req = self._zed_on_srv.Request()
        self.req.data = False
        success = self._call_service(self._zed_on_srv, self.req, "Turning Zed Off")

        self.req = self._bot_cam_on_srv.Request()
        self.req.data = False

        success = success and self._call_service(self._bot_cam_on_srv, self.req, "Turning BotCam Off")
        return success

    # private:
    # Callback methods
    def yaw_callback(self, msg: Float64) -> None:
        self.pose.yaw = msg.data

    def heave_callback(self, msg: Float64) -> None:
        self.pose.heave = msg.data

    def roll_callback(self, msg: Float64) -> None:
        self.pose.roll = msg.data

    def x_callback(self, msg: Float64) -> None:
        self.pose.x = msg.data

    def y_callback(self, msg: Float64) -> None:
        self.pose.y = msg.data
