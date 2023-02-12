#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pid_interface import PIDInterface

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

class RollControlNode(Node):
    """
    Subscribers
    - /target_pose/roll (deg)
    - /pose/roll (deg)
    - /target_twist/roll (power)
    """

    """
    Publishers
    - /output_wrench/yaw
    """
    # pid_params: PIDParams

    def __init__(self):
        super().__init__('heading_control')
        self.pid = PIDInterface("roll_pid", self.pid_callback)
        self.output_roll_pub = rospy.Publisher('/output_wrench/roll', Float64, queue_size=1)
        rospy.Subscriber('/target_pose/roll', Float64, self.target_pose_callback)
        rospy.Subscriber('/pose/roll', Float64, self.pose_callback)
        rospy.Subscriber('/target_twist/roll', Float64, self.target_twist_roll_callback)


    def target_pose_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)

    def pose_callback(self, pose: Float64):
        self.pid.set_current(pose.data)

    def target_twist_roll_callback(self, target_twist_roll: Float64):
        self.pid.disable()
        self.output_roll_pub.publish(target_twist_roll.data)

    def pid_callback(self, effort: float):
        self.output_roll_pub.publish(effort)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_roll_pub.publish(0)

if __name__ == '__main__':
    RollControlNode().run()
