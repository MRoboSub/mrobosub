#!/usr/bin/env python
import rospy
from periodic_io import PIO


if __name__ == '__main__':
    rospy.init_node('stop', anonymous=True) 
    rate = rospy.Rate(50)
    while(rospy.is_shutdown() == False):
        PIO.reset_target_twist()
        rate.sleep()
        #print("Stopping")
