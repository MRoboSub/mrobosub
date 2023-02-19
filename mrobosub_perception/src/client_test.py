#!/usr/bin/env python

import rospy
from mrobosub_msgs.srv import ObjectPosition

if __name__ == '__main__':

    rospy.init_node("ml_test", anonymous=False)
    
    rospy.wait_for_service('/object_position/bootlegger')
    
    time = rospy.get_time()
    while not rospy.is_shutdown():
        try:
            bootlegger_srv = rospy.ServiceProxy('/object_position/bootlegger', ObjectPosition)
            resp = bootlegger_srv()
            if resp.found:
                print(f"found bootlegger! confidence: {resp.confidence}")
            print(f"processed image in {rospy.get_time() - time}")
            time = rospy.get_time()
        except rospy.ServiceException as e:
            print("service call failed: %s", e)
