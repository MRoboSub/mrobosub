#!/usr/bin/env python

import rospy
from mrobosub_msgs.msg import MotorState

class MotorTestNode:
    STOP_POWER = 0

    FORWARD_POWER = 0.1
    MOTOR_TIME_S = 5
    NUM_MOTORS = 8

    def __init__(self):
        rospy.init_node("motor_test")
        self.pub = rospy.Publisher("/motor_output", MotorState, queue_size=10, latch=True)
        self.rate = rospy.Rate(1 / self.MOTOR_TIME_S)
        rospy.on_shutdown(self.stop)
    
    def run(self):
        active_motor = 0

        while not rospy.is_shutdown() and active_motor < self.NUM_MOTORS:
            msg = MotorState()
            for i in range(self.NUM_MOTORS):
                if i==active_motor:
                    setattr(msg, f"motor{i}", self.FORWARD_POWER)
                else:
                    setattr(msg, f"motor{i}", self.STOP_POWER)

            print(f"Running motor {active_motor} at {self.FORWARD_POWER}")
            self.pub.publish(msg)
            self.rate.sleep()

            active_motor += 1
    
    def stop(self):
        msg = MotorState()
        for i in range(self.NUM_MOTORS):
            setattr(msg, f"motor{i}", self.STOP_POWER)
        self.pub.publish(msg)
        print("Motor test complete")

if __name__ == "__main__":
    node = MotorTestNode()
    node.run()
    node.stop()
