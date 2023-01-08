#!/usr/bin/env python

import rospy
from lib import *

from std_msgs.msg import Float32

class TestNode(ControlLoopNode):
    iteration_rate: Param[int]
    n = SubscribedVar('/n', Float32, 0.0)

    def __init__(self, name):
        super().__init__(name)

        self.num_pub = rospy.Publisher('/num', Float32, queue_size=1)


    def loop(self):
        self.num_pub.publish(self.n.val * 10)


TestNode('test_node').run()
