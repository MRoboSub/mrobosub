#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pid_interface import PIDInterface

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

class YawControlNode(Node):
    """
    Subscribers
    - /target_pose/yaw (deg)
    - /pose/yaw (deg)
    - /target_twist/yaw (power)
    """

    """
    Publishers
    - /output_wrench/yaw
    """

    def __init__(self):
        super().__init__('heading_control')
        self.pid = PIDInterface("yaw_pid", self.pid_callback)
        self.output_yaw_pub = rospy.Publisher('/output_wrench/yaw', Float64, queue_size=1)
        rospy.Subscriber('/target_pose/yaw', Float64, self.target_pose_yaw_callback)
        rospy.Subscriber('/pose/yaw', Float64, self.pose_yaw_callback)
        rospy.Subscriber('/target_twist/yaw', Float64, self.target_twist_yaw_callback)

        
    def target_pose_yaw_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)

    def pose_yaw_callback(self, pose: Float64):
        self.pid.set_current(pose.data)

    def target_twist_yaw_callback(self, target_twist_yaw: Float64):
        self.pid.disable()
        self.output_yaw_pub.publish(target_twist_yaw.data)

    def pid_callback(self, effort: float):
        self.output_yaw_pub.publish(effort)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_yaw_pub.publish(0)

if __name__ == '__main__':
    YawControlNode().run()
