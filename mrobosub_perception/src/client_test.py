#!/usr/bin/env python

import rospy
from mrobosub_msgs.srv import ObjectPosition

if __name__ == '__main__':
    rospy.wait_for_service('/object_position/bootlegger')
    try:
        bootlegger_srv = rospy.ServiceProxy('/object_position/bootlegger', ObjectPosition)
        resp = bootlegger_srv()
    except rospy.ServiceException as e:
        print("service call failed: %s", e)