#!/usr/bin/env python

import rospy
from lib import *
from periodic_io import *

from std_msgs.msg import Float32
from std_msgs.msg import Int32

class TestNode(ControlLoopNode):
    iteration_rate: Param[int]
    subs = rospy.Subscriber('/pose/heave', Float64, _heave_callback)
    def __init__(self, name):
        super().__init__(name)
        self.pub = PIO._target_pose_heave_pub



    def loop(self):
        self.pub.publish(self.subs * 1000)


TestNode('movementTest1_node').run()