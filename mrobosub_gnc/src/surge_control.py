#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pid_interface import PIDInterface

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

class SurgeControlNode(Node):
    """
    Subscribers
    - /target_twist/surge (power)
    """

    """
    Publishers
    - /output_wrench/surge
    """
    # pid_params: PIDParams

    def __init__(self):
        super().__init__('surge_control')
        self.output_surge_pub = rospy.Publisher('/output_wrench/surge', Float64, queue_size=1)
        rospy.Subscriber('/target_twist/surge', Float64, self.target_twist_surge)

        
    def target_twist_surge(self, target_twist_surge: Float64):
        self.pub_output_surge(target_twist_surge.data)

    def pub_output_surge(self, output: float):
        self.output_surge_pub.publish(output)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_surge_pub.publish(0)

if __name__ == '__main__':
    SurgeControlNode().run()
