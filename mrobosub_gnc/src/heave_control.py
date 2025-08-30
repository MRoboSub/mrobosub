#!/usr/bin/env python
'''
Heave Control Node
Author: Nolan
Date: 2022-11-20
Last Edit: Alex Bowler
'''

import rclpy

from mrobosub_lib import Node
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

    feed_forward: float
    max_heave: float

    heave = 0

    def __init__(self):
        super().__init__('heave_control')
        self.pid = PIDInterface("heave_pid", self.pid_callback)
        self.output_heave_pub = self.create_publisher(Float64, '/output_wrench/heave', qos_profile=1)
        self.create_subscription(Float64, '/target_pose/heave', self.target_pose_callback, qos_profile=10)
        self.create_subscription(Float64, '/pose/heave', self.pose_callback, qos_profile=10)
        self.create_subscription(Float64, '/target_twist/heave', self.target_twist_heave, qos_profile=10)

        
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
            output = min(0, output)
        self.output_heave_pub.publish(output)

    def run(self): 
        # believe the main here in the init takes care of the rospy spin? Need to double check
        pass

    def cleanup(self):
        self.output_heave_pub.publish(0)

if __name__ == '__main__':
    node = HeaveControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.cleanup()