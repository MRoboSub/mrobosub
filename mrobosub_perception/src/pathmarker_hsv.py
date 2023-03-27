#!/usr/bin/env python

#pathmarker_hsv
from typing import Tuple
import cv2
import sys

from cv_bridge import CvBridge
import rospy
from mrobosub_msgs.srv import PathmarkerAngle, PathmarkerAngleResponse
from timed_service import TimedService
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node, Param
from mrobosub_perception.cfg import hsv_paramsConfig
import numpy as np

from hsv_pipeline import HsvPipeline

class PathmarkerHsv(Node):
    hsv_params: Param[dict]
    timing_threshold: Param[float]

    def __init__(self):
        super().__init__('pathmarker_hsv')

        self.br = CvBridge()

        self.always_run = rospy.myargv(sys.argv)[1] != "0" #input 1 for always_run to not have to do service calls always_run:=1

        self.sub = rospy.Subscriber('/bot_cam', Image, self.handle_frame, queue_size=1)
        self.serv = TimedService('/pathmarker_angle', PathmarkerAngle, self.timing_threshold)
        self.mask_pub = rospy.Publisher(f'/pathmarker_mask', Image, queue_size=1)
        self.annotated_pub = rospy.Publisher(f'/pathmarker_annotated', Image, queue_size=1)
        self.srv = Server(hsv_paramsConfig, self.reconfigure_callback, 'hsv_params')

    def handle_frame(self, msg):
        if(self.serv.should_run() or self.always_run):
            bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pipeline = HsvPipeline(**self.hsv_params) 
            mask = pipeline.filter_image(bgr_img, color_space=cv2.COLOR_RGB2HSV)
            detection = pipeline.find_pathmarker_object(mask)

            if detection is not None:
                l = 100
                x, y, theta = detection.x, detection.y, detection.angle
                theta_rad = np.radians(theta)
                dx, dy = int(l * np.cos(theta_rad)), int(l * np.sin(theta_rad))
                annotated_img = cv2.line(bgr_img, (x+dx,y+dy), (x-dx,y-dy), (255,0,0))
            else:
                annotated_img = bgr_img

            self.mask_pub.publish(self.br.cv2_to_imgmsg(mask, encoding='mono8'))
            self.annotated_pub.publish(self.br.cv2_to_imgmsg(annotated_img, encoding='bgr8'))
            
            response = PathmarkerAngleResponse()
            if detection is not None:
                response.found = True
                response.angle = detection.angle

            self.serv.set_result(response)
        

    def reconfigure_callback(self, config, level):
        self.hsv_params.update({k: v for k, v in config.items() if k in self.hsv_params})
        return config
    

if __name__=='__main__' :
    node = PathmarkerHsv()
    rospy.spin()
