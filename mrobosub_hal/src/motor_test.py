#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

class MotorTestNode:
    STOP_POWER = 1500

    FORWARD_POWER = 1600
    MOTOR_TIME_S = 1
    NUM_MOTORS = 8

    def __init__(self):
        rospy.init_node("motor_test")
        self.pub = rospy.Publisher("/motor", Int16MultiArray, queue_size=10, latch=True)
        self.rate = rospy.Rate(1 / self.MOTOR_TIME_S)
        rospy.on_shutdown(self.stop)
    
    def run(self):
        active_motor = 0

        while not rospy.is_shutdown() and active_motor < self.NUM_MOTORS:  
            msg = Int16MultiArray(data=[self.STOP_POWER] * self.NUM_MOTORS)
            power = self.FORWARD_POWER
            msg.data[active_motor] = power
            print(f"Running motor {active_motor} at PWM {power}")
            self.pub.publish(msg)
            self.rate.sleep()

            active_motor += 1
    
    def stop(self):
        msg = Int16MultiArray(data=[self.STOP_POWER] * self.NUM_MOTORS)
        self.pub.publish(msg)
        print("Motor test complete")

if __name__ is "__main__":
    node = MotorTestNode()
    node.run()