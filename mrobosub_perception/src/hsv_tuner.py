#!/usr/bin/env python

import rospy
from mrobosub_lib.lib import Node, Param
from dynamic_reconfigure.server import Server
from mrobosub_perception.cfg import hsv_paramsConfig
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pipeline import PathmarkerPipeline
from mrobosub_msgs.srv import PathmarkerAngleResponse
from std_msgs.msg import Float64, Bool

class HsvTuner(Node):
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
        super().__init__("hsv_tuner")
        
        self.pipeline = PathmarkerPipeline()
        self.br = CvBridge()

        self.set_pipeline_params()

        self.img_sub = rospy.Subscriber("bot_cam", Image, self.handle_frame, queue_size=1)

        self.blur_pub = rospy.Publisher('/tuner/blurred', Image, queue_size=1)
        self.hsv_pub = rospy.Publisher('/tuner/hsv_filtered', Image, queue_size=1)
        self.all_pub = rospy.Publisher('/tuner/all_lines', Image, queue_size=1)
        self.final_pub = rospy.Publisher('/tuner/filtered_lines', Image, queue_size=1)
        self.found_pub = rospy.Publisher('/tuner/found', Bool, queue_size=1)
        self.angle_pub = rospy.Publisher('/tuner/angle', Float64, queue_size=1)

        srv = Server(hsv_paramsConfig, self.reconfigure_callback)

    def set_pipeline_params(self):
        self.pipeline._hsv_threshold_hue = [self.hue_lo, self.hue_hi]
        self.pipeline._hsv_threshold_saturation = [self.sat_lo, self.sat_hi]
        self.pipeline._hsv_threshold_value = [self.val_lo, self.val_hi]
        self.pipeline._filter_lines_min_length = self.lines_min_len
        self.pipeline._filter_lines_angle = [self.lines_angle_lo, self.lines_angle_hi]

    def reconfigure_callback(self, config, level):
        for k, v in config.items():
            setattr(self, k, v)
        self.set_pipeline_params()
        return config
    
    def handle_frame(self, msg):
        image_ocv = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.pipeline.process(image_ocv)

        self.blur_pub.publish(self.br.cv2_to_imgmsg(self.pipeline.blur_output, encoding='bgr8'))
    
        mask = self.pipeline.hsv_threshold_output
        
        image = np.copy(self.pipeline.source)
        image[mask < 255] = 0
        self.hsv_pub.publish(self.br.cv2_to_imgmsg(image, encoding='bgr8'))

        image_all = np.copy(image)
        for line in self.pipeline.find_lines_output:
            image_all = cv2.line(image_all, (line[0][0], line[0][1]), (line[1][0], line[1][1]), (255,0,0), 3)
        self.all_pub.publish(self.br.cv2_to_imgmsg(image_all, encoding='bgr8'))


        image_final = np.copy(image)
        for line in self.pipeline.filter_lines_output:
            image_final = cv2.line(image_final, (line.x1, line.y1), (line.x2, line.y2), (255,0,0), 3)

        self.final_pub.publish(self.br.cv2_to_imgmsg(image_final, encoding='bgr8'))

        self.found_pub.publish(self.pipeline.found)
        self.angle_pub.publish(self.pipeline.angle)


    def run(self):
        rospy.spin()



if __name__ == '__main__':
    HsvTuner().run()



