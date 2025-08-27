#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import subprocess
import sys
import numpy as np
from dynamic_reconfigure.server import Server

from mrobosub_lib.lib import ControlLoopNode

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from mrobosub_hal.cfg import rectify_paramsConfig


class Botcam(ControlLoopNode):
    """
    Provides /bot_cam/on service and /bot_cam topic
    """ # TODO: Update

    def __init__(self) -> None:
        self.iteration_rate = 60
        super().__init__("bot_cam")
        self.device_path = sys.argv[1]
        self.on = False
        self.br = CvBridge()
        rospy.Service("/bot_cam/on", SetBool, self.handle_on_service)
        # TODO: Publish here if config param is set
        self.pub = rospy.Publisher("bot_cam", Image, queue_size=1)
        self.rectified_pub = rospy.Publisher("/rectified_image", Image, queue_size=1)
        self.f = 800
        self.w = 1920
        self.h = 1080
        self.map_x, self.map_y = self.generate_undistort_maps(self.f, self.w, self.h) # TODO: Should this be dynamic?
        self.srv = Server(rectify_paramsConfig, self.reconfigure_callback, 'rectify_params')
        self.output_w = int(1920 / 2)
        self.output_h = int(1080 / 2)


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
        self.cap.set(cv2.CAP_PROP_FOURCC, 0x47504A4D) # wtf
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 1000)
        subprocess.call(
            f"v4l2-ctl -d {self.device_path} -c white_balance_temperature_auto=0 -c brightness=-64 -c exposure_auto=1 -c exposure_absolute=4 -c contrast=50",
            shell=True,
        )

    def close_capture(self):
        self.cap.release()

    def loop(self):
        if not self.on:
            return

        success, frame = self.cap.read()

        if success:
            # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            rectified = self.undistort(frame)
            # self.pub.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
            resized = cv2.resize(rectified, (self.output_w, self.output_h))
            self.rectified_pub.publish(self.br.cv2_to_imgmsg(resized, encoding="bgr8"))

    def undistort(self, bgr_img):
        rectified_img = cv2.remap(bgr_img, self.map_x, self.map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # rectified_img = cv2.rectangle(rectified_img, (self.w // 2 - 100 - 7, self.h // 2 - 100 + 4), (self.w // 2 + 100 - 7, self.h // 2 + 100 + 4), (255, 255, 255, 3))
        # rectified_img = cv2.resize(rectified_img, (640, 480), interpolation=cv2.INTER_LINEAR)
        return rectified_img

    def reconfigure_callback(self, config, level):
        self.f = config["f"]
        self.map_x, self.map_y = self.generate_undistort_maps(self.f, self.w, self.h)
        return config

    def generate_undistort_maps(self, f, w, h):
        # The theory behind this function is that the image is distorted by a fisheye lens which produces
        # an orthogonal distortion. We need to undistort this raw image to get a rectified image.
        # See https://en.wikipedia.org/wiki/Fisheye_lens.

        # Also this just generates the maps, as the maps just rely on the image dimensions and the
        # f(ocal length), so they can be calculated ahead of time and be reused for every remap.
        # h *= 3
        # w *= 3

        # We are going to "oversample" by 3

        cx, cy = w // 2 - 8, h // 2  + 4

        # allows us to vectorize our computations
        x_u, y_u = np.meshgrid(np.arange(w), np.arange(h))

        x_rel = x_u - cx
        y_rel = y_u - cy
        x_rel *= 2
        y_rel *= 2

        #x_rel = x_rel.astype(np.float32)
        #y_rel = y_rel.astype(np.float32)

        #x_rel *= 2
        #y_rel *= 2

        # calculate the distance r_u from the center from the image
        r_u = np.sqrt(x_rel**2 + y_rel**2)

        # the angle phi from the verticle
        phi = np.arctan2(y_rel, x_rel)

        # calculate theta from r_u and f
        theta = np.arctan2(r_u, f)

        # Use the formula for the distorted radius for a orthogonal distortion
        r_d = f * np.sin(theta)

        # calculate the corresponding point in the distorted image in the distorted image
        x_d = cx + r_d * np.cos(phi)
        y_d = cy + r_d * np.sin(phi)

        map_x = x_d.astype(np.float32)
        map_y = y_d.astype(np.float32)
        return map_x, map_y



if __name__ == "__main__":
    Botcam().run()
