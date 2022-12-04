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


Namespace = Type

# Look at this!
class PIO:
#public:
    # gate_position: float
    # gman_position: float
    # bootlegger_position: float
    # gun_position: float
    yaw: float
    heave: float
    roll: float

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


    # @classmethod
    # def heading_within_threshold(cls, threshold):
    #     return angle_error_abs(PIO.heading_value, PIO.current_heading) <= threshold

    # @classmethod
    # def depth_within_threshold(cls, threshold):
    #     return (PIO.target_depth - PIO.current_depth) <= threshold

    # @classmethod
    # def set_absolute_heading(cls, heading):
    #     PIO.heading_mode = HeadingRequest.ABSOLUTE
    #     PIO.heading_value = heading

    # @classmethod
    # def set_override_heading(cls, power):
    #     PIO.heading_mode = HeadingRequest.OVERRIDE
    #     PIO.heading_value = power

    @classmethod
    def set_target_yaw(cls, target_yaw : float) -> None:
       PIO._target_yaw_pub.publish(target_yaw)

    @classmethod
    def set_target_heave(cls, target_heave: float) -> None:
        PIO._target_heave_pub.publish(target_heave)

    @classmethod
    def set_override_yaw(cls, override_yaw : float) -> None:
        PIO._yaw_pub.publish(override_yaw)
    
    @classmethod
    def set_override_surge(cls, override_surge : float) -> None:
        PIO._surge_pub.publish(override_surge)
    
    @classmethod
    def set_override_sway(cls, override_sway : float) -> None:
        PIO._sway_pub.publish(override_sway)

    @classmethod
    def set_override_roll(cls, override_roll : float) -> None:
        PIO._sway_pub.publish(override_roll)

    @classmethod
    def get_pose(cls) -> Namespace[Pose]:
        return PIO.Pose

    
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
    # rospy.Subscriber('/mrobosub/object_position/gate', ObjectPosition, _gate_position_callback)
    # rospy.Subscriber('/mrobosub/object_position/gman', ObjectPosition, _gman_position_callback)
    # rospy.Subscriber('/mrobosub/object_position/bootlegger', ObjectPosition, _bootlegger_position_callback)
    # rospy.Subscriber('/mrobosub/object_position/gun', ObjectPosition, _gun_position_callback)
    
    rospy.Subscriber('/mrobosub/pose/yaw', Float64, _yaw_callback)
    rospy.Subscriber('/mrobosub/pose/heave', Float64, _heave_callback)
    rospy.Subscriber('/mrobosub/pose/roll', Float64, _roll_callback)

    # Publishers
    _target_heave_pub = rospy.Publisher('/mrobosub/target_pos/heave', Float64, queue_size=1)
    _target_yaw_pub = rospy.Publisher('/mrobosub/target_pos/yaw', Float64, queue_size=1)
    _surge_pub = rospy.Publisher('/mrobosub/override_wrench/surge', Float64, queue_size=1)
    _sway_pub = rospy.Publisher('/mrobosub/override_wrench/sway', Float64, queue_size=1)
    _roll_pub = rospy.Publisher('/mrobosub/override_wrench/roll', Float64, queue_size=1)
    _yaw_pub = rospy.Publisher('/mrobosub/override_wrench/yaw', Float64, queue_size=1)

