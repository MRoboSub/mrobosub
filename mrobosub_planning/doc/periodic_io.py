import rospy
from std_msgs.msg import Int32


class PIO:
    @classmethod
    def _heave_cb(cls, msg):
        cls.curr_heave = msg.data

    @classmethod
    def _yaw_cb(cls, msg):
        cls.curr_yaw = msg.data

    heave_pub = rospy.Publisher('/heave_pwm', Int32, queue_size=1)
    yaw_pub = rospy.Publisher('/yaw_pwm', Int32, queue_size=1)

    rospy.Subscriber('/heave_psn', Int32, lambda m: PIO._heave_cb(m))
    rospy.Subscriber('/yaw_psn', Int32, lambda m: PIO._yaw_cb(m))

    curr_yaw: int = 0
    curr_heave: int = 0
