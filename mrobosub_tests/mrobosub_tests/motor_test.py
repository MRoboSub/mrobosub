#!/usr/bin/env python

import rclpy
from mrobosub_lib import Node

from mrobosub_msgs.msg import MotorState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MotorTest(Node):
    STOP_POWER = 0
    FORWARD_POWER = 0.1
    MOTOR_TIME_S = 5
    NUM_MOTORS = 8

    def __init__(self):
        super().__init__("motor_test")
        qos_profile = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub = self.create_publisher(MotorState, "/motor_output", qos_profile)
        self.rate = self.create_rate(1 / self.MOTOR_TIME_S)
    
    def run(self):
        active_motor = 0

        while rclpy.ok() and active_motor < self.NUM_MOTORS:
            msg = MotorState()
            for i in range(self.NUM_MOTORS):
                if i==active_motor:
                    msg.motors[i] = self.FORWARD_POWER
                else:
                    msg.motors[i] = self.STOP_POWER

            self.get_logger().info(f"Running motor {active_motor} at {self.FORWARD_POWER}")
            self.pub.publish(msg)
            self.rate.sleep()

            active_motor += 1
    
    def stop(self):
        msg = MotorState()
        for i in range(self.NUM_MOTORS):
            msg.motors[i] = self.STOP_POWER
        self.pub.publish(msg)
        self.get_logger().info("Motor test complete")
