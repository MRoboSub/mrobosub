#!/usr/bin/env python

#hsv_filter
import cv2
import sys
from cv_bridge import CvBridge
import numpy as np
import rospy
from mrobosub_msgs.srv import HsvFilterImage, HsvFilterImageResponse
from timed_service import TimedService
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node, Param
from mrobosub_perception.cfg import hsv_paramsConfig


class HsvFilter(Node):
    hue_lo: Param[float]
    hue_hi: Param[float]
    sat_lo: Param[float]
    sat_hi: Param[float]
    val_lo: Param[float]
    val_hi: Param[float]
    sub_name: Param[str]
    serv_name: Param[str]
    timing_threshold: Param[float]

    def __init__(self):
        super().__init__('hsv_filter')

        self.br = CvBridge()

        print(rospy.myargv(sys.argv))
        if(rospy.myargv(sys.argv)[1] != "0"): #input 1 for always_run to not have to do service calls always_run:=1
            self.always_run = True
        else:
            self.always_run = False

        self.sub = rospy.Subscriber(self.sub_name, Image, self.handle_frame, queue_size=1)
        #self.pub = rospy.Publisher(self.pub_name, Image, queue_size=1)
        self.serv = TimedService(self.serv_name, HsvFilterImage, self.timing_threshold)
        #print("Timed Service", self.serv)
        self.pub = rospy.Publisher(self.serv_name + '_', Image, queue_size=1)
        self.srv = Server(hsv_paramsConfig, self.reconfigure_callback)

    def handle_frame(self, msg):
        if(self.serv.should_run() or self.always_run):
            bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_HSV = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
            frame_threshold = cv2.inRange(frame_HSV, (self.hue_lo, self.sat_lo, self.val_lo), (self.hue_hi, self.sat_hi, self.val_hi))
            img = self.br.cv2_to_imgmsg(frame_threshold, encoding='mono8')
            hsv_res = HsvFilterImageResponse(img)
            self.pub.publish(img)
            self.serv.set_result(hsv_res)
        

    def reconfigure_callback(self, config, level):
        print("Recieved reconfigure")
        for k, v in config.items():
            setattr(self, k, v)
        print("Returning")
        return config

if __name__=='__main__' :
    filter = HsvFilter()
    rospy.spin()
