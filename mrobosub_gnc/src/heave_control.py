#!/usr/bin/env python

import rospy

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
    - /output_wrench/heave
    """
    # pid_params: PIDParams

    feed_forward: float
    max_heave: float

    heave = 0

    def __init__(self):
        super().__init__('heave_control')
        self.pid = PIDInterface("heave_pid", self.pid_callback)
        self.output_heave_pub = rospy.Publisher('/output_wrench/heave', Float64, queue_size=1)
        rospy.Subscriber('/target_pose/heave', Float64, self.target_pose_callback)
        rospy.Subscriber('/pose/heave', Float64, self.pose_callback)
        rospy.Subscriber('/target_twist/heave', Float64, self.target_twist_heave)

        
    def target_pose_callback(self, target_pose: Float64):
        self.pid.set_target(target_pose.data)

    def pose_callback(self, pose: Float64):
        self.heave = pose.data
        self.pid.set_current(pose.data)

    def target_twist_heave(self, target_twist_heave: Float64):
        self.pid.disable()
        self.pub_output_heave(target_twist_heave.data)

    def pid_callback(self, effort: float):
        output = effort + self.feed_forward
        self.pub_output_heave(output)

    def pub_output_heave(self, output: float):
        # Protect against running to ground
        if self.heave > self.max_heave:
            output = 0
        self.output_heave_pub.publish(output)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_heave_pub.publish(0)

if __name__ == '__main__':
    HeaveControlNode().run()
