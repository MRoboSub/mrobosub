#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pid_interface import PIDInterface

from mrobosub_lib.lib import Node, Param, signum

from typing import Optional, Final

class SurgeControlNode(Node):
    """
    Subscribers
    - /target_twist/surge (power)

    Publishers
    - /output_wrench/surge
    """
    # pid_params: PIDParams

    max_accel: float

    def __init__(self):
        super().__init__('surge_control')
        self.output_surge_pub = rospy.Publisher('/output_wrench/surge', Float64, queue_size=1)
        rospy.Subscriber('/target_twist/surge', Float64, self.target_twist_surge)

        self.prev_time = rospy.get_time()
        self.prev_output = 0

        
    def target_twist_surge(self, msg: Float64):
        desired_change = msg.data - self.prev_output
        dt = min(0.1, rospy.get_time() - self.prev_time)
        max_abs_change = self.max_accel * dt
        limited_change = signum(desired_change) * min(abs(desired_change), max_abs_change)
        output = self.prev_output + limited_change

        self.prev_output = output
        self.prev_time = rospy.get_time()
        self.pub_output_surge(output)
        

    def pub_output_surge(self, output: float):
        self.output_surge_pub.publish(output)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_surge_pub.publish(0)

if __name__ == '__main__':
    SurgeControlNode().run()
