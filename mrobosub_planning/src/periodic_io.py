import rospy
from std_msgs.msg import Float64

# TODO: where to put angle error and util repository?

# Don't look at these
def _gate_position_callback(msg: Float64):
    PIO.gate_position = msg

def _gman_position_callback(msg : Float64):
    PIO.gate_position = msg

def _bootlegger_position_callback(msg : Float64):
    PIO.gate_position = msg

def _gun_position_callback(msg : Float64):
    PIO.gun_position = msg

def _yaw_callback(msg : Float64):
    PIO.Pose.yaw = msg.data

def _heave_callback(msg : Float64):
    PIO.Pose.heave = msg.data

def _roll_callback(msg : Float64):
    PIO.Pose.roll = msg.data

# Look at this!
class PIO:
#public:

   
    # gate_position = ObjectPosition()
    # gman_position = ObjectPosition()
    # bootlegger_position = ObjectPosition()
    # gun_position = ObjectPosition()

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
    def set_target_yaw(target_yaw : float):
       PIO._target_yaw_pub.publish(target_yaw)

    @classmethod
    def set_target_heave(target_heave: float):
        PIO._target_heave_pub.publish(target_heave)

    @classmethod
    def set_override_yaw(override_yaw : float):
        PIO._yaw_pub.publish(override_yaw)
    
    @classmethod
    def set_override_surge(override_surge : float):
        PIO._surge_pub.publish(override_surge)
    
    @classmethod
    def set_override_sway(override_sway : float):
        PIO._sway_pub.publish(override_sway)

    @classmethod
    def set_override_roll(override_roll : float):
        PIO._sway_pub.publish(override_roll)

    @classmethod
    def get_pose() -> Pose:
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
    _target_heave_pub = rospy.Publisher('/mrobosub/target_pos/heave', Float64)
    _target_yaw_pub = rospy.Publisher('/mrobosub/target_pos/yaw', Float64)
    _surge_pub = rospy.Publisher('/mrobosub/output_wrench/surge', Float64)
    _sway_pub = rospy.Publisher('/mrobosub/output_wrench/sway', Float64)
    _roll_pub = rospy.Publisher('/mrobosub/override_wrench/roll', Float64)
    _yaw_pub = rospy.Publisher('/mrobosub/override_wrench/yaw', Float64)

