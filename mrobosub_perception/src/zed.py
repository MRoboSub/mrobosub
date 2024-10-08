#!/usr/bin/env python


import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy

from mrobosub_lib.lib import ControlLoopNode, Param

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import subprocess


class Zed(ControlLoopNode):
    device_path = Param[str]

    def handle_on_service(self, req: SetBoolRequest):
        if req.data == self.on: return SetBoolResponse(success=True)

        if req.data:
            self.open_capture()
        else:
            self.close_capture()
        self.on = req.data
        return SetBoolResponse(success=True)

    def open_capture(self):
        self.cap = cv2.VideoCapture(self.device_path)
        subprocess.call('v4l2-ctl -d /dev/video4 -c white_balance_temperature_auto=0 -c hue_auto=0', shell=True)

    def close_capture(self):
        self.cap.release()

    def __init__(self):
        super().__init__('zed')
        self.on = False
        self.br = CvBridge()
        self.raw_pub = rospy.Publisher('/zed/raw', Image, queue_size=1)
        rospy.Service('/zed/on', SetBool, self.handle_on_service)
        self.pub = rospy.Publisher(
            '/zed2/zed_node/rgb/image_rect_color', 
            Image, queue_size=1
        )

    def chop(self, frame):
        width = frame.shape[1]
        return frame[:,:(width//2),:]
    
    def crop(self, frame):
        left, right, top, bottom = 130, 50, 40, 60
        frame[:,:left] = frame[:,-right:] = frame[:top,:] = frame[-bottom:,:] = [255,0,0]
        return frame

    def loop(self):
        if not self.on: return

        success, frame = self.cap.read()

        if success:
            self.raw_pub.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))
            frame_chopped = self.chop(frame)
            frame_cropped = self.crop(frame_chopped)
            img = self.br.cv2_to_imgmsg(frame_cropped, encoding='bgr8')
            self.pub.publish(img)

            

if __name__ == '__main__':
    Zed().run()
