#!/usr/bin/env python

import os
import time

import torch
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from mrobosub_lib import Node, Param

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from mrobosub_msgs.msg import Detection, Detections

def load_yolo():
    # load model
    path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
    yolo_path = os.path.join(path, "src/mrobosub_perception/yolov5")

    model_path = os.path.expanduser('~/ros2_ws/src/mrobosub_perception/models/mar2026_best.pt')
    print(yolo_path)
    print(model_path)
    model = torch.hub.load(
        yolo_path, "custom", path=model_path, source="local"
    )  # local repo
    model.conf = 0.1  # NMS confidence threshold
    return model

def make_detection(d) -> Detection:
    det = Detection()
    det.left       = float(d[0])
    det.top        = float(d[1])
    det.right      = float(d[2])
    det.bottom     = float(d[3])
    det.confidence = float(d[4])
    det.classification = int(d[5])
    return det

class MlExecutor(Node):
    def __init__(self):
        super().__init__("ml_executor")

        params = [Param('run_forever', Parameter.Type.BOOL, "should ml_executor always run or only until /ml/run_until")]
        self.declare_params(params)

        self.model = load_yolo()
        self.run_until_time = float("inf") if self.run_forever else 0
        self.bridge = CvBridge()
        self.create_subscription(Image, "/zed2/zed_node/rgb/image_rect_color", self.zed_callback, qos_profile = 1)
        self.create_subscription(Float64, "/ml/run_until", self.run_until_callback, qos_profile=1)
        self.detection_pub = self.create_publisher(Detections, "/ml/detections", qos_profile=1)

    def zed_callback(self, image: Image):
        if self.get_clock().now().nanoseconds / 1e9 > self.run_until_time:
            return

        start = time.time()
        image_ocv = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

        # Find any objects in the image
        height, width, channels = image_ocv.shape
        outputs = self.model(image_ocv, size=width)  # get raw detection data

        detections = outputs.xyxy[0].cpu().numpy()

        print(f"TIME: {time.time() - start}")
        message = Detections(
            detections=[make_detection(d) for d in detections],
            width=float(width),
            height=float(height),
        )
        self.detection_pub.publish(message)

    def run_until_callback(self, message: Float64):
        self.run_until_time = max(self.run_until_time, message.data)


def main():
    rclpy.init()
    node = MlExecutor()
    rclpy.spin(node)
 
       
if __name__ == "__main__":
    main()
