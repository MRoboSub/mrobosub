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
        rospy.on_shutdown(self.stop)
    
    def run(self):
        rate = rospy.Rate(1 / self.MOTOR_TIME_S)
        activeMotor = 0

        while not rospy.is_shutdown() and activeMotor < self.NUM_MOTORS:  
            msg = Int16MultiArray(data=[self.STOP_POWER] * self.NUM_MOTORS)
            msg.data[activeMotor] = self.FORWARD_POWER
            self.pub.publish(msg)
            rate.sleep()

            activeMotor += 1
        
        self.stop()
    
    def stop(self):
        msg = Int16MultiArray(data=[self.STOP_POWER] * self.NUM_MOTORS)
        self.pub.publish(msg)

if __name__ == "__main__":
    node = MotorTestNode()
    node.run()