#!/usr/bin/env python

import os
import sys
import time

import torch
from ultralytics import YOLO
import rclpy
import numpy as np
from cv_bridge import CvBridge
from mrobosub_lib import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from mrobosub_msgs.msg import Detection, Detections

def to_detection(det: np.ndarray) -> Detection:
    msg = Detection()
    msg.left, msg.top, msg.right, msg.bottom, msg.confidence = map(float, det[:5])
    msg.classification = int(det[5])
    return msg


class MlExecutor(Node):
    def __init__(self, run_until_time: float):
        super().__init__("ml_executor")
        self.model = YOLO("yolo11n.pt")
        self.run_until_time = run_until_time
        self.bridge = CvBridge()
        self.create_subscription(Image, "/dummy_botcam", self.zed_callback, qos_profile = 1)
        self.create_subscription(Float64, "/ml/run_until", self.run_until_callback, qos_profile=1)
        self.detection_pub = self.create_publisher(Detections, "/ml/detections", qos_profile=1)

    def zed_callback(self, image: Image):
        if self.get_clock().now().nanoseconds / 1e9 > self.run_until_time:
            return

        start = time.time()
        image_ocv = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

        # Find any objects in the image
        height, width, channels = image_ocv.shape
        outputs = self.model(image_ocv, imgsz=width)  # get raw detection data

        detections = outputs[0].boxes.data.cpu().numpy()

        self.get_logger().info(f"TIME: {time.time() - start}")
        message = Detections(
            detections=([to_detection(d) for d in detections]),
            width=float(width),
            height=float(height),
        )
        self.detection_pub.publish(message)

    def run_until_callback(self, message: Float64):
        self.run_until_time = max(self.run_until_time, message.data)


def main():
    rclpy.init()
    node = MlExecutor(
        float("inf") if rclpy.utilities.remove_ros_args(sys.argv)[1] != "0" else 0,
    )
    rclpy.spin(node)
 
       
if __name__ == "__main__":
    main()
