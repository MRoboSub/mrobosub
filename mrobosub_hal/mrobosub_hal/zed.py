import cv2
from mrobosub_hal.cv_bridge_converter_hal import cv2_to_imgmsg
from sensor_msgs.msg import Image
import rclpy
import sys

from mrobosub_lib import Node

import pyzed.sl as sl

from std_srvs.srv import SetBool
import subprocess


class Zed(Node):
    """
    Provides /zed/on service and /zed/raw topic and /zed2/zed_node/rgb/image_rect_color topic
    """

    def __init__(self):
        super().__init__("zed")
        self.iteration_rate = 60
        self.device_path = sys.argv[1]
        self.on = False
        self.zed = sl.Camera()                  
        self.raw_pub = self.create_publisher(Image, "/zed/raw", qos_profile=1)
        self.create_service(SetBool, "/zed/on", self.handle_on_service)
        self.pub = self.create_publisher(Image, "/zed2/zed_node/rgb/image_rect_color", qos_profile=1)
        self.timer = self.create_timer(1.0/self.iteration_rate, self.loop)

    def handle_on_service(self, req, res):
        res.success = True
        if req.data == self.on:
            # nothing to be done, camera is already in the desired state
            return res

        if req.data:
            self.open_capture()
        else:
            self.close_capture()
        self.on = req.data

        return res

    def open_capture(self):
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720    # Use HD720 video mode for USB cameras
        init_params.camera_fps = 60     
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(repr(err))
            print(repr(err))
            exit(-1)
    
        self.runtime_param = sl.RuntimeParameters()
        
        # self.cap = cv2.VideoCapture(self.device_path)
        # subprocess.call(
        #     f"v4l2-ctl -d {self.device_path} -c white_balance_temperature_auto=0 -c hue_auto=0",
        #     shell=True,
        # )

    def close_capture(self):
        self.zed.close()
        # self.cap.release()

    def chop(self, frame):
        width = frame.shape[1]
        return frame[:, : (width // 2), :]

    def crop(self, frame):
        left, right, top, bottom = 130, 50, 40, 60
        frame[:, :left] = frame[:, -right:] = frame[:top, :] = frame[-bottom:, :] = [
            255,
            0,
            0,
        ]
        return frame

    def loop(self):
        if not self.on:
            return
        
        image = sl.Mat()
        depth = sl.Mat()
        
        # Display a pixel color
        self.zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
        frame = image.get_data()                         # (H, W, 4) uint8, BGRA
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # -> (H, W, 3) BGR
        self.raw_pub.publish(cv2_to_imgmsg(frame, encoding="bgr8"))
        
        frame_chopped = self.chop(frame)
        frame_cropped = self.crop(frame_chopped)
        img = cv2_to_imgmsg(frame_cropped, encoding="bgr8")
        self.pub.publish(img)

        # Display a pixel depth
        self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH) # Get the depth map
        
        
        # success, frame = self.cap.read()

        # if success:
        #     self.raw_pub.publish(cv2_to_imgmsg(frame, encoding="bgr8"))
        #     frame_chopped = self.chop(frame)
        #     frame_cropped = self.crop(frame_chopped)
        #     img = cv2_to_imgmsg(frame_cropped, encoding="bgr8")
        #     self.pub.publish(img)


def main():
    rclpy.init()
    node = Zed()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

