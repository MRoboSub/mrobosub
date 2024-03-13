#hsv_filter
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, ImageResponse
from mrobosub_lib.lib import Node, Param

class HsvFilter(Node):
    hue_lo: Param[float]
    hue_hi: Param[float]
    sat_lo: Param[float]
    sat_hi: Param[float]
    val_lo: Param[float]
    val_hi: Param[float]
    topic_name: Param[str]

    def _init_(self):
        super().__init__('hsv_filter')
        self.service = rospy.Service(self.topic_name+"/out", Image, self.handle_request)
        rospy.Subscriber(self.topic_name+"/in", Image, self.handle_frame, queue_size=1)

        self.response = ImageResponse()

    def handle_request(self, _msg):


    def handle_frame(self, data):

