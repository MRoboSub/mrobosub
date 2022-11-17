import rospy
from mrobosub_control.msg import ObjectPosition, HeadingRequest
from std_msgs.msg import Float32
from util.control_motor import MotorController
from util.angles import angle_error_abs

# TODO: where to put angle error and util repository?

# Don't look at these
def _gate_position_callback(msg):
    PeriodicIO.gate_position = msg

def _gman_position_callback(msg):
    PeriodicIO.gate_position = msg

def _bootlegger_position_callback(msg):
    PeriodicIO.gate_position = msg

def _gun_position_callback(msg):
    PeriodicIO.gun_position = msg

def _heading_callback(msg):
    PeriodicIO.current_heading = msg.data

def _depth_callback(msg):
    PeriodicIO.current_depth = msg.data

def _roll_callback(msg):
    PeriodicIO.current_roll = msg.data

# Look at this!
class PeriodicIO:
#public:

    # Inputs
    current_heading = 0
    current_roll = 0
    current_depth = 0
    gate_position = ObjectPosition()
    gman_position = ObjectPosition()
    bootlegger_position = ObjectPosition()
    gun_position = ObjectPosition()

    # Output
    heading_mode = HeadingRequest.DISABLED
    heading_value = 0
    target_depth = 0
    forward = 0
    lateral = 0
    roll = 0

    @classmethod
    def heading_within_threshold(cls, threshold):
        return angle_error_abs(PeriodicIO.heading_value, PeriodicIO.current_heading) <= threshold

    @classmethod
    def depth_within_threshold(cls, threshold):
        return (PeriodicIO.target_depth - PeriodicIO.current_depth) <= threshold

    @classmethod
    def set_absolute_heading(cls, heading):
        PeriodicIO.heading_mode = HeadingRequest.ABSOLUTE
        PeriodicIO.heading_value = heading

    @classmethod
    def set_override_heading(cls, power):
        PeriodicIO.heading_mode = HeadingRequest.OVERRIDE
        PeriodicIO.heading_value = power

    @classmethod
    def publish_all(cls):
        # PeriodicIO._mc.set(
        #     forward=PeriodicIO.forward,
        #     lateral=PeriodicIO.lateral
        # )
        PeriodicIO._foward_pub.publish(PeriodicIO.forward)
        PeriodicIO._strafe_pub.publish(PeriodicIO.lateral)
        PeriodicIO._heading_request_pub.publish(
            HeadingRequest(PeriodicIO.heading_mode, PeriodicIO.heading_value)
        )
        PeriodicIO._depth_request_pub.publish(PeriodicIO.target_depth)
        PeriodicIO._roll_pub.publish(PeriodicIO.roll)

# private:

    # _mc = MotorController()

    # Subscribers
    rospy.Subscriber('/mrobosub/object_position/gate', ObjectPosition, _gate_position_callback)
    rospy.Subscriber('/mrobosub/object_position/gman', ObjectPosition, _gman_position_callback)
    rospy.Subscriber('/mrobosub/object_position/bootlegger', ObjectPosition, _bootlegger_position_callback)
    rospy.Subscriber('/mrobosub/object_position/gun', ObjectPosition, _gun_position_callback)
    rospy.Subscriber('/mrobosub/heading', Float32, _heading_callback)
    rospy.Subscriber('/mrobosub/altitude', Float32, _depth_callback)
    rospy.Subscriber('/mrobosub/roll', Float32, _roll_callback)

    # Publishers
    _depth_request_pub = rospy.Publisher('/mrobosub/depth_request', Float32)
    _heading_request_pub = rospy.Publisher('/mrobosub/heading_request', HeadingRequest)
    _foward_pub = rospy.Publisher('/mrobosub/out/forward', Float32)
    _strafe_pub = rospy.Publisher('/mrobosub/out/strafe', Float32)
    _roll_pub = rospy.Publisher('/mrobosub/out/roll', Float32)

