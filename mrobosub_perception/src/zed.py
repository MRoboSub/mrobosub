#!/usr/bin/env python


import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy

from mrobosub_lib.lib import ControlLoopNode, Param




class Zed(ControlLoopNode):
    device_path = Param[str]

    def __init__(self):
        super().__init__('zed')
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(self.device_path)
        self.raw_pub = rospy.Publisher('/zed/raw', Image, queue_size=1)
        self.pub = rospy.Publisher(
            '/zed2/zed_node/rgb/image_rect_color', 
            Image, queue_size=1
        )

    def chop(self, frame):
        width = frame.shape[1]
        return frame[:,:(width//2),:]

    def loop(self):
        success, frame = self.cap.read()

        if success:
            self.raw_pub.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))
            frame_chopped = self.chop(frame)
            img = self.br.cv2_to_imgmsg(frame_chopped, encoding='bgr8')
            self.pub.publish(img)

            

if __name__ == '__main__':
    Zed().run()
