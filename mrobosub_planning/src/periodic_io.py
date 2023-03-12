import rospy
from std_msgs.msg import Float64

from typing import Type

# TODO: where to put angle error and util repository?

# Don't look at these
# def _gate_position_callback(msg: Float64) -> None:
#     PIO.gate_position = msg
#
# def _gman_position_callback(msg : Float64) -> None:
#     PIO.gate_position = msg
#
# def _bootlegger_position_callback(msg : Float64) -> None:
#     PIO.gate_position = msg
#
# def _gun_position_callback(msg : Float64) -> None:
#     PIO.gun_position = msg
#
def _yaw_callback(msg : Float64) -> None:
    PIO.Pose.yaw = msg.data

def _heave_callback(msg : Float64) -> None:
    PIO.Pose.heave = msg.data

def _roll_callback(msg : Float64) -> None:
    PIO.Pose.roll = msg.data

def angle_error(setpoint, state):
        return (setpoint - state + 180) % 360 - 180

Namespace = Type

# Look at this!
class PIO:
    # TODO: gun_position, buoy_position, gate_position, bootlegger_position, set_absolute_heading

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
        yaw = 0
        heave = 0
        roll = 0

    class TargetPose:
        yaw = 0
        heave = 0
        roll = 0

    # @classmethod
    # def heading_within_threshold(cls, threshold):
    #     return angle_error_abs(PIO.heading_value, PIO.current_heading) <= threshold

    @classmethod
    def is_yaw_within_threshold(cls, threshold):
        return abs(angle_error(cls.TargetPose.yaw, cls.Pose.yaw)) <= threshold
    

    # @classmethod
    # def depth_within_threshold(cls, threshold):
    #     return (PIO.target_depth - PIO.current_depth) <= threshold

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
    def get_pose(cls) -> Namespace[Pose]:
        return cls.Pose
    
    @classmethod
    def have_seen_pathmaker():
        pass


    
    # @classmethod
    # def publish_all(cls):
    #     # PIO._mc.set(
    #     #     forward=PIO.forward,
    #     #     lateral=PIO.lateral
    #     # )
    #     PIO._foward_pub.publish(PIO.forward)
    #     PIO._strafe_pub.publish(PIO.lateral)
    #     PIO._heading_request_pub.publish(
    #         HeadingRequest(PIO.heading_mode, PIO.heading_value)
    #     )
    #     PIO._depth_request_pub.publish(PIO.target_depth)
    #     PIO._roll_pub.publish(PIO.roll)

# private:

    # _mc = MotorController()

    # Subscribers
    # rospy.Subscriber('/object_position/gate', ObjectPosition, _gate_position_callback)
    # rospy.Subscriber('/object_position/gman', ObjectPosition, _gman_position_callback)
    # rospy.Subscriber('/object_position/bootlegger', ObjectPosition, _bootlegger_position_callback)
    # rospy.Subscriber('/object_position/gun', ObjectPosition, _gun_position_callback)
    
    rospy.Subscriber('/pose/yaw', Float64, _yaw_callback)
    rospy.Subscriber('/pose/heave', Float64, _heave_callback)
    rospy.Subscriber('/pose/roll', Float64, _roll_callback)

    # Publishers
    _target_pose_heave_pub = rospy.Publisher('/target_pose/heave', Float64, queue_size=1)
    _target_pose_yaw_pub = rospy.Publisher('/target_pose/yaw', Float64, queue_size=1)
    _target_pose_roll_pub = rospy.Publisher('/target_pose/roll', Float64, queue_size=1)

    _target_twist_yaw_pub = rospy.Publisher('/target_twist/yaw', Float64, queue_size=1)
    _target_twist_roll_pub = rospy.Publisher('/target_twist/roll', Float64, queue_size=1)
    _target_twist_surge_pub = rospy.Publisher('/target_twist/surge', Float64, queue_size=1)
    _target_twist_sway_pub = rospy.Publisher('/target_twist/sway', Float64, queue_size=1)
