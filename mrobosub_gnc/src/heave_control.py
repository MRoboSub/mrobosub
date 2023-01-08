#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pid_interface import PIDInterface

from mrobosub_lib.lib import Node, Param

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
    - /output_wrench/yaw
    """
    # pid_params: PIDParams

    feed_forward: float

    def __init__(self):
        super().__init__('heading_control')
        rospy.Subscriber('/target_pose/heave', Float64, self.target_pose_callback)
        rospy.Subscriber('/pose/heave', Float64, self.pose_callback)
        rospy.Subscriber('/target_twist/heave', Float64, self.target_twist_heave)
        self.output_heave_pub = rospy.Publisher('/output_wrench/heave', Float64, queue_size=1)

        self.pid = PIDInterface("heave_pid", self.pid_callback)
        
    def target_pose_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)

    def pose_callback(self, pose: Float64):
        self.pid.set_current(pose.data)

    def target_twist_heave(self, target_twist_heave: Float64):
        self.pid.disable()
        self.output_heave_pub.publish(target_twist_heave.data)

    def pid_callback(self, effort: float):
        output = effort + self.feed_forward
        self.output_heave_pub.publish(output)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_heave_pub.publish(0)

if __name__ == '__main__':
    HeaveControlNode().run()
