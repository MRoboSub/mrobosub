#!/usr/bin/env python

import cv2

from cv_bridge import CvBridge
from utils import crop_to_circle, generate_rectify_maps
import rospy
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node
from mrobosub_perception.cfg import rectify_paramsConfig

class RectifiedImage(Node):
    def __init__(self):
        super().__init__('rectified_image')
        self.f = 280

        self.br = CvBridge()
        self.map_x, self.map_y = None, None
        self.h, self.w = 640, 480
        self.shape = None
        
        self.sub = rospy.Subscriber('/dummy_botcam', Image, self.handle_frame, queue_size=1)
        self.rectified_pub = rospy.Publisher(f'/rectified_image', Image, queue_size=1)
        self.srv = Server(rectify_paramsConfig, self.reconfigure_callback, 'rectify_params')

    def handle_frame(self, msg):
        bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.map_x is None or self.map_y is None:
            # Generate the maps only once, as they are static for the given focal length and image size
            self.h, self.w = bgr_img.shape[:2]
            self.map_x, self.map_y = generate_rectify_maps(self.h, self.w, self.f)

        rectified_img = cv2.remap(bgr_img, self.map_x, self.map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        rectified_img = cv2.rectangle(rectified_img, (self.w // 2 - 100 - 7, self.h // 2 - 100 + 4), (self.w // 2 + 100 - 7, self.h // 2 + 100 + 4), (255, 255, 255, 3))
        #rectified_img = cv2.resize(rectified_img, (640, 480), interpolation=cv2.INTER_LINEAR)
        self.rectified_pub.publish(self.br.cv2_to_imgmsg(rectified_img, encoding='bgr8'))

    def reconfigure_callback(self, config, level):
        self.f = config["f"]
        self.map_x, self.map_y = generate_rectify_maps(self.h, self.w, self.f)
        return config

    def run(self):
        rospy.spin()

if __name__== '__main__':
    RectifiedImage().run()

