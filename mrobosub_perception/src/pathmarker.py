#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node
from cv_bridge import CvBridge
import cv2
from mrobosub_msgs.srv import PathmarkerAngle, PathmarkerAngleResponse
import numpy as np

from mrobosub_lib.lib import Node, Param

from pipeline import PathmarkerPipeline


class Pathmarker(Node):
    hue_lo: Param[float]
    hue_hi: Param[float]
    sat_lo: Param[float]
    sat_hi: Param[float]
    val_lo: Param[float]
    val_hi: Param[float]
    lines_min_len: Param[float]
    lines_angle_lo: Param[float]
    lines_angle_hi: Param[float]

    def __init__(self):
        super().__init__('pathmarker')
        self.service = rospy.Service('pathmarker/angle', PathmarkerAngle, handle_request)
        rospy.Subscriber("bot_cam", Image, self.handle_frame, queue_size=1)

        self.pipeline = PathmarkerPipeline()
        self.br = CvBridge()

        self.pipeline.__hsv_threshold_hue = [self.hue_lo, self.hue_hi]
        self.pipeline.__hsv_threshold_saturation = [self.sat_lo, self.sat_hi]
        self.pipeline.__hsv_threshold_value = [self.val_lo, self.val_hi]
        self.pipeline.__filter_lines_min_length = self.lines_min_len
        self.pipeline.__filter_lines_angle = [self.lines_angle_lo, self.lines_angle_hi]

        self.resp = PathmarkerAngleResponse()


    def handle_frame(self, data):
        image_ocv = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')

        self.pipeline.process(image_ocv)
        self.resp.found = pipeline.found
        self.resp.angle = pipeline.angle


    def handle_request(self, msg):
        return self.resp

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    Pathmarker().run()
