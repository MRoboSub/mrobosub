#!/usr/bin/env python

#pathmarker_hsv
from typing import Tuple
import cv2
import sys

from cv_bridge import CvBridge
import rclpy.utilities
from mrobosub_msgs.srv import PathmarkerAngle
from timed_service import TimedService
from sensor_msgs.msg import Image
from mrobosub_lib import Node
#from rclpy.node import Node

import numpy as np

from hsv_pipeline import HsvPipeline
class PathmarkerHsv(Node):
    
    def __init__(self,args):
        super().__init__('pathmarker_hsv')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hsv_params.ros__parameters.hue_lo',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.hue_hi',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.sat_lo',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.sat_hi',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.val_lo',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.val_hi',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.wb_shift',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.wb_scale',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.white_balance',rclpy.Parameter.Type.BOOL),
                ('hsv_params.ros__parameters.histogram_equalization',rclpy.Parameter.Type.BOOL),
                ('hsv_params.ros__parameters.erode_radius',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.dilate_radius',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.median_radius',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.gaussian_radius',rclpy.Parameter.Type.INTEGER),
                ('hsv_params.ros__parameters.timing_threshold',rclpy.Parameter.Type.DOUBLE)
            ])
        self.br = CvBridge()

        self.always_run = [1] != "0" #input 1 for always_run to not have to do service calls always_run:=1

        #self.sub = rospy.Subscriber('/rectified_image', Image, self.handle_frame, queue_size=1)
        self.sub = self.create_subscription(Image, '/rectified_image', self.handle_frame, 1)
        timing_threshold = self.get_parameter('hsv_params.ros__parameters.timing_threshold').get_parameter_value().double_value;
        
        self.serv = TimedService(PathmarkerAngle,'/pathmarker_angle',timing_threshold)

        #self.mask_pub = rospy.Publisher(f'/pathmarker_mask', Image, queue_size=1)
        self.mask_pub = self.create_publisher({Image}, '/pathmarker_mask', qos_profile = 1)

        #self.annotated_pub = rospy.Publisher(f'/pathmarker_annotated', Image, queue_size=1)
        self.annotated_pub = self.create_publisher({Image}, '/pathmarker_annotated', qos_profile = 1)
        
    
    def handle_frame(self, msg):
        if(self.serv.should_run() or self.always_run):
            bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #pipeline = HsvPipeline(**self.hsv_params, color_space=cv2.COLOR_RGB2HSV) 
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

            self.mask_pub.publish(self.br.cv2_to_imgmsg(mask, encoding='mono8'))
            self.annotated_pub.publish(self.br.cv2_to_imgmsg(annotated_img, encoding='bgr8'))
            
            response = PathmarkerAngle.Response()
            if detection is not None:
                response.found = True
                response.angle = detection.angle
                response.centroid_x = detection.x / bgr_img.shape[1]
                response.centroid_y = detection.y / bgr_img.shape[0]

            self.serv.set_result(response)
    

if __name__=='__main__' :
    args = rclpy.utilities.remove_ros_args(sys.argv)
    node = PathmarkerHsv(args)
    rclpy.spin(node)
