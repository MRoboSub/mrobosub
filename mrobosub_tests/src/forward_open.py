#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

ITERATION_RATE = 100 #Hz

def angle_error(setpoint, state):
    return (setpoint - state + 180) % 360 - 180

class ForwardOpen(Node):

    def __init__(self):
        super().__init__('forward_open')
        self.heave_target_pub = rospy.Publisher('/target_pose/heave', Float64, queue_size=1)
        self.heave_target_twist_pub = rospy.Publisher('/target_twist/heave', Float64, queue_size=1)
        self.surge_output_pub = rospy.Publisher('/target_twist/surge', Float64, queue_size=1)
        self.rate = rospy.Rate(ITERATION_RATE)

    def run(self): 
        try:
            print('starting depth request')
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < 5.0 and not rospy.is_shutdown():
                self.heave_target_pub.publish(0.35)
                self.rate.sleep()
            start_time = rospy.get_time()
            print('starting surge')
            while rospy.get_time() - start_time < 5.0 and not rospy.is_shutdown():
                self.surge_output_pub.publish(0.2)
                self.rate.sleep()
            print('done')
        except KeyboardInterrupt():
            for _ in range(100):
                self.surge_output_pub.publish(0)
                self.heave_target_twist_pub.publish(0)
                self.rate.sleep()

    def cleanup(self):
        self.surge_output_pub.publish(0)
        self.heave_target_twist_pub.publish(0)

if __name__ == '__main__':
    ForwardOpen().run()
