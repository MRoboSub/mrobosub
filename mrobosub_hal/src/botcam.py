#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import subprocess
import sys

from mrobosub_lib.lib import ControlLoopNode

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


class Botcam(ControlLoopNode):
    def __init__(self) -> None:
        self.iteration_rate = 60
        super().__init__("bot_cam")
        self.device_path = sys.argv[1]
        self.on = False
        self.br = CvBridge()
        rospy.Service("/bot_cam/on", SetBool, self.handle_on_service)
        self.pub = rospy.Publisher("bot_cam", Image, queue_size=1)

    def handle_on_service(self, req: SetBoolRequest):
        if req.data == self.on:
            return SetBoolResponse(success=True)

        if req.data:
            self.open_capture()
        else:
            self.close_capture()
        self.on = req.data
        return SetBoolResponse(success=True)

    def open_capture(self):
        self.cap = cv2.VideoCapture(self.device_path)
        # https://stackoverflow.com/a/66279297
        # cap.set(cv2.CAP_PROP_FPS,10)
        # cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 1000)
        subprocess.call(
            f"v4l2-ctl -d {self.device_path} -c white_balance_temperature_auto=0 -c brightness=64 -c exposure_auto=3",
            shell=True,
        )

    def close_capture(self):
        self.cap.release()

    def loop(self):
        if not self.on:
            return

        success, frame = self.cap.read()

        if success:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            self.pub.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))


if __name__ == "__main__":
    Botcam().run()
