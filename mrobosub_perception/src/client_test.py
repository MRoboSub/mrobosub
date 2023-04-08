#!/usr/bin/env python

import rospy
from mrobosub_msgs.srv import ObjectPosition

if __name__ is '__main__':

    rospy.init_node("ml_test", anonymous=False)
    
    rospy.wait_for_service('/object_position/gun')
    
    time = rospy.get_time()
    while not rospy.is_shutdown():
        print("finished run")
        try:
            bootlegger_srv = rospy.ServiceProxy('/object_position/gun', ObjectPosition)
            resp = bootlegger_srv()
            if resp.found:
                print(f"found gun! confidence: {resp.confidence}")
            print(f"processed image in {rospy.get_time() - time}")
            time = rospy.get_time()
        except rospy.ServiceException as e:
            print("service call failed: %s", e)
