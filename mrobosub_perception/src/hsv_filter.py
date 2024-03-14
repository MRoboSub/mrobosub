#hsv_filter
import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node, Param

class HsvFilter(Node):
    hue_lo: Param[float]
    hue_hi: Param[float]
    sat_lo: Param[float]
    sat_hi: Param[float]
    val_lo: Param[float]
    val_hi: Param[float]
    sub_name: Param[str]
    pub_name: Param[str]

    def _init_(self):
        super().__init__('hsv_filter')

        self.br = CvBridge()

        self.sub = rospy.Subscriber(self.sub_name, Image, self.handle_frame, queue_size=1)
        self.pub = rospy.Publisher(self.pub_name, Image, queue_size=1)
        

    def handle_frame(self, msg):
        bgr_img = self.br.imgmsg_tocv2(msg, desired_encoding='bgr8')
        frame_HSV = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        frame_threshold = cv2.inRange(frame_HSV, (self.hue_lo, self.sat_lo, self.val_lo), (self.hue_hi, self.sat_hi, self.val_hi))
        self.pub.publish(self.br.cv2_to_imgmsg(frame_threshold, encoding='mono8'))
        

