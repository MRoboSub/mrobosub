#!/usr/bin/env python

import cv2

from cv_bridge import CvBridge
from utils import crop_to_circle, generate_rectify_maps
import rospy
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node
from mrobosub_perception.cfg import hsv_paramsConfig


class RectifiedImage(Node):
    f: int 
    crop_radius: int 

    def __init__(self):
        super().__init__('rectified_image')
        self.f = 596
        self.crop_radius = 610

        self.br = CvBridge()
        
        self.sub = rospy.Subscriber('/bot_cam', Image, self.handle_frame, queue_size=1)
        self.rectified_pub = rospy.Publisher(f'/rectified_image', Image, queue_size=1)
        self.srv = Server(hsv_paramsConfig, self.reconfigure_callback, 'hsv_params')
        self.map_x, self.map_y = None, None

    def handle_frame(self, msg):
        bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cropped_img = crop_to_circle(bgr_img, self.crop_radius)

        if self.map_x is None or self.map_y is None:
            # Generate the maps only once, as they are static for the given focal length and image size
            self.map_x, self.map_y = generate_rectify_maps(cropped_img, self.f)

        rectified_img = cv2.remap(cropped_img, self.map_x, self.map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        self.rectified_pub.publish(self.br.cv2_to_imgmsg(rectified_img, encoding='bgr8'))

    def reconfigure_callback(self, config, level):
        self.f = config["f"]
        self.crop_radius = config["crop_radius"]
        return config

    def run(self):
        rospy.spin()

if __name__=='__main__' :
    RectifiedImage().run()

