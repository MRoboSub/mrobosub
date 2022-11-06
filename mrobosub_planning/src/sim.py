import rospy
from std_msgs.msg import Int32

class Submarine:
    req_heave = 0
    req_yaw = 0
    heave = 0
    yaw = 0
    heave_publisher = rospy.Publisher('/heave_psn', Int32, queue_size=1)
    yaw_publisher = rospy.Publisher('/yaw_psn', Int32, queue_size=1)

    @classmethod
    def simulate_timestep(cls, timestep: float):
        cls.heave = max(0, cls.heave + cls.req_heave * timestep)
        cls.yaw = (cls.yaw + cls.req_yaw * timestep / 10) % 360 - 180
        cls.heave_publisher.publish(Int32(cls.heave))
        cls.yaw_publisher.publish(Int32(cls.yaw))

    @classmethod
    def heave_cb(cls, msgs):
        cls.req_heave = msgs.data

    @classmethod
    def yaw_cb(cls, msgs):
        cls.req_yaw = msgs.data

RATE = 100
rospy.init_node('submarine_sim')
rate_manager = rospy.Rate(RATE)
heave_subscriber = rospy.Subscriber('/heave_pwm',  Int32, Submarine.req_heave)
yaw_subscriber = rospy.Subscriber('/yaw_pwm',  Int32, Submarine.req_yaw)

while not rospy.is_shutdown():
    Submarine.simulate_timestep(1 / RATE)
    rate_manager.sleep()
