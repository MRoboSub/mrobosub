#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pid_interface import PIDInterface

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

class PitchControlNode(Node):
    """
    Subscribers
    - /target_pose/pitch (deg)
    - /pose/pitch (deg)
    - /target_twist/pitch (power)
    """

    """
    Publishers
    - /output_wrench/pitch
    """
    # pid_params: PIDParams

    def __init__(self):
        super().__init__('heading_control')
        self.pid = PIDInterface("pitch_pid", self.pose_pid_callback)
        #REVIEW - Added twist_pid
        self.twist_pid = PIDInterface("twist_pid", self.twist_pid_callback)
        self.output_pitch_pub = rospy.Publisher('/output_wrench/pitch', Float64, queue_size=1)
        #REVIEW - Added output_twist_pitch_pub
        self.output_twist_pitch_pub = rospy.Publisher('/output_twist/heave', Float64, queue_size=1)
        rospy.Subscriber('/target_pose/pitch', Float64, self.target_pose_callback)
        rospy.Subscriber('/pose/pitch', Float64, self.pose_callback)
        #REVIEW - Added target_twist/pitch and twist/pitch
        rospy.Subscriber('/target_twist/pitch', Float64, self.target_twist_callback)
        rospy.Subscriber('/twist/pitch', Float64, self.twist_callback)

    
    def target_pose_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)

    #REVIEW - Added target_twist_pitch_callback  
    def target_twist_callback(self, target_twist: Float64):
        # self.pid.disable()
        self.twist_pid.set_target(target_twist.data)


    def pose_callback(self, pose: Float64):
        self.pid.set_current(pose.data)
    #REVIEW - Added twist_callback
    def twist_callback(self, twist: Float64):
        self.pid.set_current(twist.data)

    def pose_pid_callback(self, effort: float):
        self.output_pitch_pub.publish(effort)

    #REVIEW - Added twist_pid_callback
    def twist_pid_callback(self, effort: float):
        self.output_twist_pitch_pub(effort)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_pitch_pub.publish(0)
        self.output_twist_pitch_pub(0)
if __name__ == '__main__':
    PitchControlNode().run()
