#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

ITERATION_RATE = 100 #Hz

def angle_error(setpoint, state):
    return (setpoint - state + 180) % 360 - 180

class RectangleTestNoTurnNode(Node):

    submerge_wait: Param[float]
    target_heave: Param[float]
    surge_output: Param[float]
    before_turn_wait: Param[float]
    angle_threshold: Param[float]

    class Pose:
        yaw = 0
        heave = 0

    def __init__(self):
        super().__init__('rectangle_test_no_turn')
        rospy.Subscriber('/pose/yaw', Float64, self.pose_yaw_callback)
        rospy.Subscriber('/pose/heave', Float64, self.pose_heave_callback)
        self.yaw_target_pub = rospy.Publisher('/target_pose/yaw', Float64, queue_size=1)
        self.heave_target_pub = rospy.Publisher('/target_pose/heave', Float64, queue_size=1)
        self.yaw_twist_pub = rospy.Publisher('/target_twist/yaw', Float64, queue_size=1)
        self.surge_output_pub = rospy.Publisher('/output_wrench/surge', Float64, queue_size=1)
        self.sway_output_pub = rospy.Publisher('/output_wrench/swap', Float64, queue_size=1)
        self.rate = rospy.Rate(ITERATION_RATE)
    
    def pose_yaw_callback(self, yaw: Float64):
        self.Pose.yaw = yaw

    def pose_heave_callback(self, heave: Float64):
        self.Pose.heave = heave

    def close_to(self, setpoint):
        return abs(angle_error(setpoint, self.Pose.yaw)) <= self.angle_threshold

    def run(self): 
        print('starting depth request')
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < self.submerge_wait and not rospy.is_shutdown():
            self.heave_target_pub.publish(self.target_heave)
            self.rate.sleep()

        print('starting square')
        self.square(side_time=5, control_heading_on_forward=True, repeat=4)

    def square(self, side_time=10, control_heading_on_forward=True, repeat=4):
        """ moves forwards and turns the given angle 4 times. """
        start_yaw = self.Pose.yaw
        target_yaw = start_yaw

        OUTPUTS = [
            [1, 0],
            [0, 1],
            [-1, 0],
            [0, -1]
        ]

        for i in range(repeat):
            surge_output = self.surge_output * OUTPUTS[i][0]
            sway_output = self.surge_output * OUTPUTS[i][0]

            start_time = rospy.get_time()

            # forward
            while rospy.get_time() - start_time < side_time and not rospy.is_shutdown():
                self.surge_output_pub.publish(surge_output)
                self.sway_output_pub.publish(sway_output)

                self.heave_target_pub.publish(self.target_heave)
                if control_heading_on_forward:
                    self.yaw_target_pub.publish(target_yaw)
                else:
                    self.yaw_twist_pub.publish(0)
                self.rate.sleep()
            
            self.surge_output_pub.publish(0)
            self.sway_output_pub.publish(0)
            rospy.sleep(self.before_turn_wait)

    def cleanup(self):
        self.yaw_twist_pub.publish(0)
        self.surge_output_pub.publish(0)
        self.sway_output_pub.publish(0)

if __name__ is '__main__':
    RectangleTestNoTurnNode().run()
