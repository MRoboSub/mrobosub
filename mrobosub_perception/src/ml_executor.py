#!/usr/bin/env python

import os
import sys
import time

import torch
import rospy
import numpy as np
from cv_bridge import CvBridge
from mrobosub_lib.lib import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from mrobosub_msgs.msg import Detection, Detections


def load_yolo():
    # load model
    path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    yolo_path = os.path.join(path, "yolov5")

    model_path = os.path.join(path, "models/comp_gate_2024.pt")
    model = torch.hub.load(
        yolo_path, "custom", path=model_path, source="local"
    )  # local repo
    model.conf = 0.1  # NMS confidence threshold
    return model


class MlExecutor(Node):
    def __init__(self):
        super().__init__("ml_executor")
        self.model = load_yolo()
        self.run_until_time = float("inf") if rospy.myargv(sys.argv)[1] != "0" else 0
        self.bridge = CvBridge()
        rospy.Subscriber(
            "/zed2/zed_node/rgb/image_rect_color", Image, self.zed_callback
        )
        rospy.Subscriber("/ml/run_until", Float64, self.run_until_callback)
        self.detection_pub = rospy.Publisher("/ml/detections", Detections, queue_size=1)

    def zed_callback(self, image: Image):
        if rospy.get_time() > self.run_until_time:
            return

        start = time.time()
        image_ocv = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

        # Find any objects in the image
        height, width, channels = image_ocv.shape
        outputs = self.model(image_ocv, size=width)  # get raw detection data

        detections = outputs.xyxy[0].cpu().numpy()

        print(f"TIME: {time.time() - start}")
        message = Detections(
            detections=(Detection(*d) for d in detections),
            width=width,
            height=height,
        )
        self.detection_pub.publish(message)

    def run_until_callback(self, message: Float64):
        self.run_until_time = max(self.run_until_time, message.data)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    MlExecutor().run()
