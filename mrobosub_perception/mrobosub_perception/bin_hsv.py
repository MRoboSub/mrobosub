#!/usr/bin/env python

#hsv_filter
from typing import Tuple
import cv2
import sys

from cv_bridge import CvBridge
import rclpy
from rclpy import utilities
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse
from timed_service import TimedService
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node

from hsv_pipeline import HsvPipeline
import utils

class BinHsv(Node):
    timing_threshold: Param[float]

    def __init__(self, always_run: bool):
        super().__init__('bin_hsv')

        self.declare_params()

        self.br = CvBridge()

        self.always_run = always_run

        self.sub = self.create_subcriber(Image, '/rectified_image', self.handle_frame, qos_profile=1)
        self.serv = TimedService(self, '/bin_object_position', ObjectPosition, self.timing_threshold)
        self.mask_pub = self.create_publisher(Image, f'/bin_mask', qos_profile=1)
        self.enhanced_pub = self.create_publisher(Image, f'/bin_enhanced', qos_profile=1)
        self.annotated_pub = self.create_publisher(Image, f'/bin_annotated', qos_profile=1)

    def handle_frame(self, msg):
        if(self.serv.should_run() or self.always_run):
            bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pipeline = HsvPipeline(**self.hsv_params, color_space=cv2.COLOR_BGR2HSV)
            mask, enhanced_img = pipeline.filter_image(bgr_img, return_enhanced=True)
            detection = pipeline.find_circular_object(mask)

            if detection is not None:
                annotated_img = cv2.drawMarker(bgr_img, (detection.x,detection.y), (255,255,255), markerType=cv2.MARKER_CROSS)
            else:
                annotated_img = bgr_img

            self.mask_pub.publish(self.br.cv2_to_imgmsg(mask, encoding='mono8'))
            self.enhanced_pub.publish(self.br.cv2_to_imgmsg(enhanced_img, encoding='bgr8'))
            self.annotated_pub.publish(self.br.cv2_to_imgmsg(annotated_img, encoding='bgr8'))
            
            response = ObjectPositionResponse()
            if detection is not None:
                x_theta, y_theta = utils.pixels_to_angles(bgr_img, detection.x, detection.y)
                response.found = True
                response.x_position = detection.x / bgr_img.shape[1]
                response.y_position = detection.y / bgr_img.shape[0]
                response.x_theta = x_theta
                response.y_theta = y_theta

            self.serv.set_result(response)

    def declare_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('timing_threshold', rclpy.Parameter.Type.DOUBLE),
                ('int_number', None),
                ('float_number', None),
                ('str_text', None),
                ('bool_array', None),
                ('int_array', None),
                ('float_array', None),
                ('str_array', None),
                ('bytes_array', None),
                ('nested_param.another_int', None)
            ])
        self.declare_parameter(
            "timing_threshold",
        )
        self.declare_parameter(
            "hsv_params.hue_lo", rclpy.Parameter.Type.INTEGER_ARRAY)
        
    
def main():
    rclpy.init()
    node = BinHsv(rclpy.utilities.remove_ros_args(sys.argv)[1] != "0")
    rclpy.spin(node)


if __name__=='__main__' :
    main()
