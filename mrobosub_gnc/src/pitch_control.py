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
        self.pid = PIDInterface("pitch_pid", self.pid_callback)
        self.output_pitch_pub = rospy.Publisher('/output_wrench/pitch', Float64, queue_size=1)
        rospy.Subscriber('/target_pose/pitch', Float64, self.target_pose_callback)
        rospy.Subscriber('/pose/pitch', Float64, self.pose_callback)
        rospy.Subscriber('/target_twist/pitch', Float64, self.target_twist_pitch_callback)

        
    def target_pose_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)

    def pose_callback(self, pose: Float64):
        self.pid.set_current(pose.data)

    def target_twist_pitch_callback(self, target_twist_pitch: Float64):
        self.pid.disable()
        self.output_pitch_pub.publish(target_twist_pitch.data)

    def pid_callback(self, effort: float):
        self.output_pitch_pub.publish(effort)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_pitch_pub.publish(0)

if __name__ == '__main__':
    PitchControlNode().run()
