#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pid_interface import PIDInterface

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

class RollControlNode(Node):
    """
    Subscribers
    - /mrobosub/target_pose/roll (deg)
    - /mrobosub/pose/roll (deg)
    - /mrobosub/override_wrench/roll (power)
    """

    """
    Publishers
    - /mrobosub/output_wrench/yaw
    """
    # pid_params: PIDParams

    def __init__(self):
        super().__init__('heading_control')
        rospy.Subscriber('/mrobosub/target_pose/roll', Float64, self.target_pose_callback)
        rospy.Subscriber('/mrobosub/pose/roll', Float64, self.pose_callback)
        rospy.Subscriber('/mrobosub/override_wrench/roll', Float64, self.override_wrench_callback)
        self.output_roll_pub = rospy.Publisher('/mrobosub/output_wrench/roll', Float64, queue_size=1)

        self.pid = PIDInterface("roll_pid", self.pid_callback)
        
    def target_pose_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)

    def pose_callback(self, pose: Float64):
        self.pid.set_current(pose.data)

    def override_wrench_callback(self, override_wrench: Float64):
        self.output_roll_pub.publish(override_wrench.data)

    def pid_callback(self, effort: float):
        self.output_roll_pub.publish(effort)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_roll_pub.publish(0)

if __name__ == '__main__':
    RollControlNode().run()
