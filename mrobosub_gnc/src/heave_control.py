#!/usr/bin/env python

import rospy

from mrobosub_lib.lib import Node, Param
from std_msgs.msg import Float64
from pid_interface import PIDInterface

from typing import Optional, Final

class HeaveControlNode(Node):
    """
    Subscribers
    - /target_pose/heave (deg)
    - /pose/heave (deg)
    - /target_twist/heave (power)
    """

    """
    Publishers
    - /output_wrench/heave
    """
    # pid_params: PIDParams

    feed_forward: float
    max_heave: float

    heave = 0

    def __init__(self):
        super().__init__('heave_control')
        self.pid = PIDInterface("heave_pid", self.pose_pid_callback)
        self.twist_pid = PIDInterface("twist_pid", self.twist_pid_callback)
        self.output_heave_pub = rospy.Publisher('/output_wrench/heave', Float64, queue_size=1)
        # REVIEW - Added output_twist_heave_pub
        self.output_twist_heave_pub = rospy.Publisher('/output_twist/heave', Float64, queue_size=1)
        rospy.Subscriber('/target_pose/heave', Float64, self.target_pose_callback)
        rospy.Subscriber('/pose/heave', Float64, self.pose_callback)
        # REVIEW - Added target_twist/heave and twist/heave
        rospy.Subscriber('/target_twist/heave', Float64, self.target_twist_heave_callback)
        rospy.Subscriber('/twist/heave', Float64, self.twist_callback)

        
    def target_pose_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)

    def pose_callback(self, pose: Float64):
        self.heave = pose.data
        self.pid.set_current(pose.data)

    # REVIEW - Added target_twist_heave_callback and twist_callback
    def target_twist_heave_callback(self, target_twist_heave: Float64):
        # self.pid.disable()
        self.twist_pid.set_target(target_twist_heave.data)

    def twist_callback(self, twist: Float64):
        self.twist_pid = twist.data
        self.pid.set_current(twist.data)

    def pose_pid_callback(self, effort: float):
        output = effort + self.feed_forward
        self.pub_output_heave(output)
        
    # REVIEW - Added twist_pid_callback
    def twist_pid_callback(self, effort: float):
        self.feed_forward = effort
        self.pid.set_feed_forward(effort)
    

    def pub_output_heave(self, output: float):
        # Protect against running to ground
        if self.heave > self.max_heave:
            output = min(0, output)
        self.output_heave_pub.publish(output)

    def output_twist_heave(self, output: float):
        # Protect against running to ground
        if self.heave > self.max_heave:
            output = min(0, output)
        self.output_twist_heave_pub.publish(output)


    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_heave_pub.publish(0)
        self.output_twist_heave_pub.publish(0)

if __name__ == '__main__':
    HeaveControlNode().run()
