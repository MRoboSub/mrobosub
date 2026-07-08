#!/usr/bin/env python

import os
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from mrobosub_lib import Node, Param
from ultralytics import YOLO

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from mrobosub_msgs.msg import SegDetection, SegDetections

# NMS confidence threshold applied inside the model itself, same role as
# model.conf in ml_executor.py's yolov5 loader.
MODEL_CONFIDENCE = 0.1


def load_yolo():
    # load model
    path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
    model_path = os.path.join(path, "models/temp2026_seg_best.pt")
    print(model_path)
    model = YOLO(model_path)
    return model


class SegExecutor(Node):
    def __init__(self):
        super().__init__("seg_executor")

        params = [Param('run_forever', Parameter.Type.BOOL, "should seg_executor always run or only until /seg/run_until")]
        self.declare_params(params)

        self.model = load_yolo()
        self.run_until_time = float("inf") if self.run_forever else 0
        self.bridge = CvBridge()
        self.create_subscription(Image, "/rectified_image", self.image_callback, qos_profile=1)
        self.create_subscription(Float64, "/seg/run_until", self.run_until_callback, qos_profile=1)
        self.detection_pub = self.create_publisher(SegDetections, "/seg/detections", qos_profile=1)

    def image_callback(self, image: Image):
        if self.get_clock().now().nanoseconds / 1e9 > self.run_until_time:
            return

        start = time.time()
        image_ocv = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

        # Find any objects in the image
        height, width, channels = image_ocv.shape
        results = self.model(image_ocv, imgsz=width, conf=MODEL_CONFIDENCE, verbose=False)  # get raw detection data
        r = results[0]

        seg_detections = []
        if r.masks is not None:
            # r.masks.xy: list of Nx2 arrays, one polygon per instance, in pixel coords of the ORIGINAL image
            # r.boxes.cls / r.boxes.conf: per-instance class id / confidence, same ordering as r.masks.xy
            polygons_px = r.masks.xy
            classes = r.boxes.cls.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()

            for polygon, cls, conf in zip(polygons_px, classes, confs):
                points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in polygon]
                seg_detections.append(
                    SegDetection(polygon=points, confidence=float(conf), classification=int(cls))
                )

        print(f"TIME: {time.time() - start}")
        message = SegDetections(
            detections=seg_detections,
            width=float(width),
            height=float(height),
        )
        self.detection_pub.publish(message)

    def run_until_callback(self, message: Float64):
        self.run_until_time = max(self.run_until_time, message.data)


def main():
    rclpy.init()
    node = SegExecutor()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
