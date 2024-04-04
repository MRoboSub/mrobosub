#!/usr/bin/env python

#hsv_filter
import cv2
import sys

from matplotlib.pyplot import annotate
from cv_bridge import CvBridge
import numpy as np
import rospy
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse
from timed_service import TimedService
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node, Param
from mrobosub_perception.cfg import hsv_paramsConfig

from hsv_pipeline import HsvPipeline

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
        self.serv = TimedService(self.serv_name, ObjectPosition, self.timing_threshold)
        #print("Timed Service", self.serv)
        self.mask_pub = rospy.Publisher(f'{self.serv_name}_mask', Image, queue_size=1)
        self.annotated_pub = rospy.Publisher(f'{self.serv_name}_annotated', Image, queue_size=1)
        self.srv = Server(hsv_paramsConfig, self.reconfigure_callback)

    def handle_frame(self, msg):
        if(self.serv.should_run() or self.always_run):
            bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pipeline = HsvPipeline(
                int(self.hue_lo), int(self.hue_hi), int(self.sat_lo),
                int(self.sat_hi), int(self.val_lo), int(self.val_hi))
            mask = pipeline.filter_image(bgr_img)
            detection = pipeline.find_circular_object(mask)
            
            if detection is not None:
                annotated_img = cv2.circle(bgr_img, (detection.x,detection.y), int(detection.radius), (255,255,255), 2)
            else:
                annotated_img = bgr_img

            self.mask_pub.publish(self.br.cv2_to_imgmsg(mask, encoding='mono8'))
            self.annotated_pub.publish(self.br.cv2_to_imgmsg(annotated_img, encoding='bgr8'))
            
            response = ObjectPositionResponse()
            if detection is not None:
                response.found = True
                response.x_position = detection.x
                response.y_position = detection.y

            self.serv.set_result(response)
        

    def reconfigure_callback(self, config, level):
        print("Recieved reconfigure")
        for k, v in config.items():
            setattr(self, k, v)
        print("Returning")
        return config

if __name__=='__main__' :
    filter = HsvFilter()
    rospy.spin()
