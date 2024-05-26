#!/usr/bin/env python

#hsv_filter
from typing import Tuple
import cv2
import sys

from cv_bridge import CvBridge
import rospy
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse
from timed_service import TimedService
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node, Param
from mrobosub_perception.cfg import hsv_paramsConfig

from hsv_pipeline import HsvPipeline
import utils

class BinHsv(Node):
    hsv_params: Param[dict]
    timing_threshold: Param[float]

    def __init__(self):
        super().__init__('buoy_hsv')

        self.br = CvBridge()

        self.always_run = rospy.myargv(sys.argv)[1] != "0" #input 1 for always_run to not have to do service calls always_run:=1

        self.sub = rospy.Subscriber('/bot_cam', Image, self.handle_frame, queue_size=1)
        self.serv = TimedService('/bin_object_position', ObjectPosition, self.timing_threshold)
        self.mask_pub = rospy.Publisher(f'/bin_mask', Image, queue_size=1)
        self.enhanced_pub = rospy.Publisher(f'/bin_enhanced', Image, queue_size=1)
        self.annotated_pub = rospy.Publisher(f'/bin_annotated', Image, queue_size=1)
        self.srv = Server(hsv_paramsConfig, self.reconfigure_callback, 'hsv_params')

    def handle_frame(self, msg):
        if(self.serv.should_run() or self.always_run):
            bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pipeline = HsvPipeline(**self.hsv_params, color_space=cv2.COLOR_RGB2HSV)
            mask, enhanced_img = pipeline.filter_image(bgr_img, return_enhanced=True)
            detection = pipeline.find_rectangular_object(mask)

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
                response.x_position = detection.x
                response.y_position = detection.y
                response.x_theta = x_theta
                response.y_theta = y_theta

            self.serv.set_result(response)
        

    def reconfigure_callback(self, config, level):
        self.hsv_params.update({k: v for k, v in config.items() if k in self.hsv_params})
        return config
    

if __name__=='__main__' :
    node = BinHsv()
    rospy.spin()
