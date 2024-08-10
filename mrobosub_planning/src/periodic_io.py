import rosgraph
import rospy
from std_msgs.msg import Float64, Bool, Int32
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse, PathmarkerAngle # type: ignore
from typing import Dict, Type, Mapping, Optional, Tuple
from enum import Enum, auto
from std_srvs.srv import SetBool


def angle_error(setpoint, state):
        return (((setpoint - state) % 360) + 360) % 360

Namespace = Type

class ImageTarget(Enum):
    GATE_BLUE = auto()
    GATE_RED = auto()

ImageDetections = Dict[ImageTarget, ObjectPositionResponse]

# Look at this!
class PIO:
#public:
    # gate_position: float
    # gman_position: float
    # bootlegger_position: float
    # gun_position: float

    # # Output
    # heading_mode = HeadingRequest.DISABLED
    # heading_value = 0
    # target_depth = 0
    # forward = 0
    # lateral = 0
    # roll = 0

    class Pose:
        yaw = 0.0
        heave = 0.0
        roll = 0.0

    class TargetPose:
        yaw = 0.0
        heave = 0.0
        roll = 0.0
    
    buoy_collision = False


    # @classmethod
    # def heading_within_threshold(cls, threshold):
    #     return angle_error_abs(PIO.heading_value, PIO.current_heading) <= threshold

    @classmethod
    def query_BinCamPos(cls) -> Optional[ObjectPositionResponse]:
        """ Request the x, y position on the camera of the bin (0,0) being the center +y is up and +x is right,
          and found which is True if we have data

        Returns: 
            x, y position on the camera of the bin and found
            None otherwise. 
        """
        
        try:
            resp = cls._bin_cam_pos_srv()
        except rospy.service.ServiceException as e:
            print('Cannot reach bin object position service')
            return None
        if(resp.found):
            return resp
        else:
            return None


    @classmethod
    def is_yaw_within_threshold(cls, threshold):
        return abs(angle_error(cls.TargetPose.yaw, cls.Pose.yaw)) <= threshold

    @classmethod
    def is_heave_within_threshold(cls, threshold):
        return abs(cls.TargetPose.heave - cls.Pose.heave) <= threshold

    # @classmethod
    # def set_absolute_heading(cls, heading):
    #     PIO.heading_mode = HeadingRequest.ABSOLUTE
    #     PIO.heading_value = heading

    @classmethod
    def set_target_pose_yaw(cls, target_yaw : float) -> None:
       cls._target_pose_yaw_pub.publish(target_yaw)
       cls.TargetPose.yaw = target_yaw

    @classmethod
    def set_target_pose_heave(cls, target_heave: float) -> None:
        cls._target_pose_heave_pub.publish(target_heave)
        cls.TargetPose.heave = target_heave

    @classmethod
    def set_target_pose_roll(cls, target_roll: float) -> None:
        cls._target_pose_roll_pub.publish(target_roll)
        cls.TargetPose.roll = target_roll

    @classmethod
    def set_target_twist_roll(cls, override_roll : float) -> None:
        cls._target_twist_roll_pub.publish(override_roll)

    @classmethod
    def set_target_twist_yaw(cls, override_yaw : float) -> None:
        cls._target_twist_yaw_pub.publish(override_yaw)
    
    @classmethod
    def set_target_twist_surge(cls, override_surge : float) -> None:
        cls._target_twist_surge_pub.publish(override_surge)
    
    @classmethod
    def set_target_twist_sway(cls, override_sway : float) -> None:
        cls._target_twist_sway_pub.publish(override_sway)
    
    @classmethod
    def set_target_twist_heave(cls, override_heave: float) -> None:
        cls._target_twist_heave_pub.publish(override_heave)

    @classmethod
    def reset_target_twist(cls) -> None:
        cls.set_target_twist_heave(0)
        cls.set_target_twist_yaw(0)
        cls.set_target_twist_surge(0)
        cls.set_target_twist_roll(0)
        cls.set_target_twist_sway(0)

    @classmethod
    def set_left_dropper_angle(cls, angle: int) -> None:
        cls._left_dropper_pub.publish(angle)
    
    @classmethod
    def set_right_dropper_angle(cls, angle: int) -> None:
        cls._right_dropper_pub.publish(angle)

    # @classmethod
    # def get_pose(cls) -> Namespace[Pose]:
    #     return cls.Pose
        
    @classmethod
    def query_pathmarker(cls) -> Optional[float]:
        """ Request a the pathmaker angle.

        Returns: 
            angle of path marker in global frame (i.e. same frame as the Pose.yaw) if found
            None otherwise. 
        """
        
        try:
            resp = cls._pathmarker_srv()
        except rospy.service.ServiceException as e:
            print('Pathmarker service is not active')
            return None
        print(f"{resp=}")
        if resp.found:
            convertedAngle = (90 + resp.angle) + cls.Pose.yaw
            if convertedAngle > 90: #if pointing behind us flip 180
                convertedAngle -= 180
            elif convertedAngle < -90:
                convertedAngle += 180
            return convertedAngle
        else:
            return None

    @classmethod
    def query_buoy(cls) -> ObjectPositionResponse:
        try:
            return cls._hsv_buoy_position_srv()
        except rospy.ServiceException as e:
            print(f'Buoy service cannot be called with error: {e}')
            obj_msg = ObjectPositionResponse()
            obj_msg.found = False
            return obj_msg

    @classmethod
    def query_image(cls, image_type: Optional[ImageTarget]) -> ObjectPositionResponse:
        if image_type is not None:
            try:
                return cls._object_position_srvs[image_type]()
            except rospy.ServiceException:
                pass
        obj_msg = ObjectPositionResponse()
        obj_msg.found = False
        return obj_msg
    
    @classmethod
    def query_all_images(cls) -> ImageDetections:
        results = { }
        for g in ImageTarget:
            resp = cls.query_image(g)
            if resp.found:
                results[g] = resp
        return results
    
    
    @classmethod
    def activate_zed(cls):
        try:
            cls._bot_cam_on_srv(False)
        except rospy.service.ServiceException as e:
            print(f'Error: Cannot deactivate Bot Cam, {e}')
        try:
            cls._zed_on_srv(True)
        except rospy.service.ServiceException as e:
            print(f'Error: Cannot activate ZED, {e}')

    @classmethod
    def activate_bot_cam(cls):
        try:
            cls._zed_on_srv(False)
        except rospy.service.ServiceException as e:
            print(f'Error: Cannot deactivate ZED, {e}')
        try:
            cls._bot_cam_on_srv(True)
        except rospy.service.ServiceException as e:
            print(f'Error: Cannot activate Bot Cam, {e}')

    @classmethod
    def deactivate_cameras(cls):
        try:
            cls._zed_on_srv(False)
        except rospy.service.ServiceException as e:
            print(f'Error: Cannot deactivate ZED, {e}')
        try:
            cls._bot_cam_on_srv(False)
        except rospy.service.ServiceException as e:
            print(f'Error: Cannot deactivate Bot Cam, {e}')

    @classmethod
    def _set_cameras(cls, zed_on: bool, bot_cam_on: bool, error_msg="Cannot call camera service"):
        try:
            cls._zed_on_srv(zed_on)
            cls._bot_cam_on_srv(bot_cam_on)
        except rospy.service.ServiceException as e:
            print(f'Error: {error_msg}, {e}')


# private:
    class Callbacks:
        @staticmethod
        def yaw_callback(msg : Float64) -> None:
            PIO.Pose.yaw = msg.data

        @staticmethod
        def heave_callback(msg : Float64) -> None:
            PIO.Pose.heave = msg.data

        @staticmethod
        def roll_callback(msg : Float64) -> None:
            PIO.Pose.roll = msg.data

        @staticmethod
        def collision_callback(msg : Bool) -> None:
            PIO.buoy_collision = msg.data

    # Subscribers
    rospy.Subscriber('/pose/yaw', Float64, Callbacks.yaw_callback)
    rospy.Subscriber('/pose/heave', Float64, Callbacks.heave_callback)
    rospy.Subscriber('/pose/roll', Float64, Callbacks.roll_callback)
    rospy.Subscriber('/collision/collision', Bool, Callbacks.collision_callback)


    # Publishers
    _target_pose_heave_pub = rospy.Publisher('/target_pose/heave', Float64, queue_size=1)
    _target_pose_yaw_pub = rospy.Publisher('/target_pose/yaw', Float64, queue_size=1)
    _target_pose_roll_pub = rospy.Publisher('/target_pose/roll', Float64, queue_size=1)

    _target_twist_yaw_pub = rospy.Publisher('/target_twist/yaw', Float64, queue_size=1)
    _target_twist_roll_pub = rospy.Publisher('/target_twist/roll', Float64, queue_size=1)
    _target_twist_surge_pub = rospy.Publisher('/target_twist/surge', Float64, queue_size=1)
    _target_twist_sway_pub = rospy.Publisher('/target_twist/sway', Float64, queue_size=1)
    _target_twist_heave_pub = rospy.Publisher('/target_twist/heave', Float64, queue_size = 1)

    _left_dropper_pub = rospy.Publisher('/left_servo/angle', Int32)
    _right_dropper_pub = rospy.Publisher('/right_servo/angle', Int32)
    
    # Services
    _pathmarker_srv = rospy.ServiceProxy('/pathmarker_angle', PathmarkerAngle, persistent=True)
    _bin_cam_pos_srv = rospy.ServiceProxy('/bin_object_position', ObjectPosition, persistent = True)
    _hsv_buoy_position_srv = rospy.ServiceProxy('/buoy_object_position', ObjectPosition, persistent=True)

    #also old
    _object_position_srvs: Dict[ImageTarget, rospy.ServiceProxy] = {}
    for glyph in ImageTarget:
        _object_position_srvs[glyph] = rospy.ServiceProxy(f'/object_position/{glyph.name.lower()}', ObjectPosition, persistent=True)

    _zed_on_srv = rospy.ServiceProxy('/zed/on', SetBool, persistent=True)
    _bot_cam_on_srv = rospy.ServiceProxy('/bot_cam/on', SetBool, persistent=True)
