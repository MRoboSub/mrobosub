#!/usr/bin/env python

#pathmarker_hsv
from typing import Tuple
import cv2
import sys

import rclpy.utilities
from rclpy.parameter import Parameter
from mrobosub_msgs.srv import PathmarkerAngle
from mrobosub_perception.timed_service import TimedService
from sensor_msgs.msg import Image
from mrobosub_lib import Node, Param
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
from mrobosub_perception.cv_bridge_converter import cv2_to_imgmsg, imgmsg_to_cv2

from mrobosub_perception.hsv_pipeline import HsvPipeline
class PathmarkerHsv(Node):
    
    def __init__(self):
        super().__init__('hsv_pathmarker')

        params = [Param('timing_threshold', Parameter.Type.DOUBLE, "A float parameter")]
        self.declare_params(params)

        args_ros = rclpy.utilities.remove_ros_args(sys.argv)
        self.always_run = args_ros[1] != "0" #input 1 for always_run to not have to do service calls always_run:=1

        self.sub = self.create_subscription(Image, '/rectified_image', self.handle_frame, 1)
        
        self.serv = TimedService(self,'/pathmarker_angle',PathmarkerAngle, self.timing_threshold)

        self.mask_pub = self.create_publisher(Image, '/pathmarker_mask', qos_profile = 1)

        self.annotated_pub = self.create_publisher(Image, '/pathmarker_annotated', qos_profile = 1)
        
    
    def handle_frame(self, msg):
        if(self.serv.should_run() or self.always_run):
            bgr_img = imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pipeline = HsvPipeline(**self._parameters, color_space=cv2.COLOR_RGB2HSV) 
            mask = pipeline.filter_image(bgr_img) 
            detection = pipeline.find_pathmarker_object(mask)

            annotated_img = bgr_img
            if detection is not None:
                l = 100
                x, y, theta = detection.x, detection.y, detection.angle
                theta_rad = np.radians(theta)
                try:
                    dx, dy = int(l * np.cos(theta_rad)), int(l * np.sin(theta_rad))
                    annotated_img = cv2.line(bgr_img, (x+dx,y+dy), (x-dx,y-dy), (255,0,0))
                except ValueError as e:
                    pass

            self.mask_pub.publish(cv2_to_imgmsg(mask, encoding='mono8'))
            self.annotated_pub.publish(cv2_to_imgmsg(annotated_img, encoding='bgr8'))
            
            response = PathmarkerAngle.Response()
            if detection is not None:
                response.found = True
                response.angle = detection.angle
                response.centroid_x = detection.x / bgr_img.shape[1]
                response.centroid_y = detection.y / bgr_img.shape[0]

            self.serv.set_result(response)
       
    
def main():
    rclpy.init()
    node = PathmarkerHsv()
    rclpy.spin(node)

if __name__=='__main__' :
    main()
