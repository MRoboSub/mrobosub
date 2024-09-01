#!/usr/bin/env python

import rospy
from mrobosub_lib.lib import *

from std_msgs.msg import Float32

class TestNode(ControlLoopNode):
    iteration_rate: Param[int]

    def __init__(self, name):
        super().__init__(name)

        self.num_pub = rospy.Publisher('/num', Float32, queue_size=1)
        self.n = 0

    def loop(self):
        self.num_pub.publish(self.n * 10)

    @subscriber('/n')
    def number(self, msg: Float32):
        self.n = msg.data


TestNode('test_node').run()
