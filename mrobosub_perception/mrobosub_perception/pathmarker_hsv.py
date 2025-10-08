#!/usr/bin/env python

#pathmarker_hsv
from typing import Tuple
import cv2
import sys

from cv_bridge import CvBridge
import rclpy.utilities
from mrobosub_msgs.srv import PathmarkerAngle
from mrobosub_perception.timed_service import TimedService
from sensor_msgs.msg import Image
from mrobosub_lib import Node
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np

from mrobosub_perception.hsv_pipeline import HsvPipeline
class PathmarkerHsv(Node):
    
    def __init__(self):
        super().__init__('hsv_pathmarker')

        self.declare_and_get_params()
        
        self.br = CvBridge()
        args_ros = rclpy.utilities.remove_ros_args(sys.argv)
        self.always_run = args_ros[1] != "0" #input 1 for always_run to not have to do service calls always_run:=1

        #self.sub = rospy.Subscriber('/rectified_image', Image, self.handle_frame, queue_size=1)
        self.sub = self.create_subscription(Image, '/rectified_image', self.handle_frame, 1)
        
        self.serv = TimedService(self,'/pathmarker_angle',PathmarkerAngle, self.timing_threshold)

        #self.mask_pub = rospy.Publisher(f'/pathmarker_mask', Image, queue_size=1)
        self.mask_pub = self.create_publisher(Image, '/pathmarker_mask', qos_profile = 1)

        #self.annotated_pub = rospy.Publisher(f'/pathmarker_annotated', Image, queue_size=1)
        self.annotated_pub = self.create_publisher(Image, '/pathmarker_annotated', qos_profile = 1)

        self.get_logger().info(f"{self.timing_threshold=}")
        
    
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

    def declare_and_get_params(self):
        param_desc_float = ParameterDescriptor()
        param_desc_float.type = rclpy.Parameter.Type.DOUBLE
        param_desc_float.description = "A float parameter"
        param_desc_bool = ParameterDescriptor()
        param_desc_bool.type = rclpy.Parameter.Type.BOOL
        param_desc_bool.description = "A bool parameter"

        self.declare_parameters(
            namespace='',
            parameters=[
                # Dont need these for now but is prob needed somewhere else and i wana copy paste this and not have to rewrite
                # ('hue_lo',0.0, param_desc_float),
                # ('hue_hi',0.0, param_desc_float),
                # ('sat_lo',0.0, param_desc_float),
                # ('sat_hi',0.0, param_desc_float),
                # ('val_lo',0.0, param_desc_float),
                # ('val_hi',0.0, param_desc_float),
                # ('wb_shift',0.0, param_desc_float),
                # ('wb_scale',0.0, param_desc_float),
                # ('white_balance',False, param_desc_bool),
                # ('histogram_equalization',False, param_desc_bool),
                # ('erode_radius',0.0, param_desc_float),
                # ('dilate_radius',0.0, param_desc_float),
                # ('median_radius',0.0, param_desc_float),
                # ('gaussian_radius',0.0, param_desc_float),
                ('timing_threshold',0.0, param_desc_float)
            ])
        
        self.timing_threshold = self.get_parameter('timing_threshold').get_parameter_value().double_value
    
def main():
    rclpy.init()
    node = PathmarkerHsv()
    rclpy.spin(node)
if __name__=='__main__' :
    main()
