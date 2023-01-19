#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float34

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

class DepthTranslationNode(Node):
    """
    Subscribers
    - /depth/raw_depth
    """

    """
    Publishers
    - /pose/heave
    """
    # pid_params: PIDParams

    def __init__(self):
        super().__init__('depth_estimation')
        rospy.Subscriber('/depth/raw_depth', Float32, self.raw_depth_callback)
        self.depth_pub = rospy.Publisher('/pose/heave', Float64, queue_size=1)
        
    def raw_depth_callback(self, raw_depth: Float32):
        self.depth_pub.publish(raw_depth.data)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        pass

if __name__ == '__main__':
    DepthTranslationNode().run()
