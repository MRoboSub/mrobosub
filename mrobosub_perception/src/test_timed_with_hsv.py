#!/usr/bin/env python

from hsv_filter import HsvFilter
from timed_service import TimedService
from sensor_msgs.msg import Image
import rospy

hsvTimed = TimedService("/TimedHSVService", Image, Image, 1.5)


def handle_frame(msg):
    if(hsvTimed.should_run()):
        hsvTimed.set_result(msg)


sub = rospy.Subscriber("/hsv_filter_output", Image, handle_frame)


t=3
while(t<30):
    
    t+=3
    rospy.sleep(3)