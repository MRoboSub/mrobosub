#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pid_interface import PIDInterface

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

class SwayControlNode(Node):
    """
    Subscribers
    - /target_twist/sway (power)
    """

    """
    Publishers
    - /output_wrench/yaw
    """
    # pid_params: PIDParams

    def __init__(self):
        super().__init__('surge_control')
        self.output_sway_pub = rospy.Publisher('/output_wrench/sway', Float64, queue_size=1)
        rospy.Subscriber('/target_twist/sway', Float64, self.target_twist_sway)

        
    def target_twist_sway(self, target_twist_sway: Float64):
        self.pub_output_sway(target_twist_sway.data)

    def pub_output_sway(self, output: float):
        self.output_sway_pub.publish(output)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_sway_pub.publish(0)

if __name__ is '__main__':
    SwayControlNode().run()
