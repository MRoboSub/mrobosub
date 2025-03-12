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
    - /output_wrench/roll
    """
    # pid_params: PIDParams

    def __init__(self):
        super().__init__('heading_control')
        self.pid = PIDInterface("roll_pid", self.pid_callback)
        #REVIEW - Added twist_pid
        self.twist_pid = PIDInterface("twist_roll_pid", self.pid_callback)
        self.output_roll_pub = rospy.Publisher('/output_wrench/roll', Float64, queue_size=1)
        #REVIEW - Added output_twist_roll_pub
        self.output_twist_roll_pub = rospy.Publisher('/output_twist/roll', Float64, queue_size=1)
        rospy.Subscriber('/target_pose/roll', Float64, self.target_pose_callback)
        rospy.Subscriber('/pose/roll', Float64, self.pose_callback)
        #REVIEW - Added target_twist/roll
        rospy.Subscriber('/target_twist/roll', Float64, self.target_twist_callback)
        rospy.Subscriber('/twist/roll', Float64, self.twist_callback)

    def target_pose_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)
    #REVIEW - Added target_twist_callback
    def target_twist_callback(self, target_twist_roll: Float64):
        # self.pid.disable()
        self.output_roll_pub.publish(target_twist_roll.data)

    def pose_callback(self, pose: Float64):
        self.pid.set_current(pose.data)

    #REVIEW - Added twist_callback
    def twist_callback(self, twist):
        self.pid.set_current(twist)


    def pid_callback(self, effort: float):
        self.output_roll_pub.publish(effort)
    #REVIEW - Added twist_pid_callback
    def twist_pid_callback(self,effort):
        self.output_twist_roll_pub.publish(effort)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_roll_pub.publish(0)

if __name__ == '__main__':
    RollControlNode().run()
