#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from mrobosub_lib.lib import Node, Param

class ForwardTestNode(Node):
    

    def __init__(self):
       super().__init__('forward_test')
       _surge_pub = rospy.Publisher('/mrobosub/output_wrench/surge', Float64)
       

       

    def run(self):
        time = rospy.get_time()
        while rospy.get_time() < time + 20:
            self._surge_pub.publish(0.4)

    def cleanup(self):
        self._surge_pub.publish(0)

if __name__ == '__main__':
    ForwardTestNode().run()
