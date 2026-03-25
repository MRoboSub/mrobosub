#!/usr/bin/env python

from typing import List, Optional
from enum import Enum

import rclpy
import numpy as np
import cv2
from cv_bridge import CvBridge
from mrobosub_lib import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from mrobosub_msgs.msg import Detections
from mrobosub_msgs.srv import ObjectPosition

CONFIDENCE = 0.5
TIME_THRESHOLD = 10

class Targets(Enum):
    SHARK = 0
    SAWFISH = 1
    BIN_SHARK = 2
    BIN_SAWFISH = 3
    GATE_BACK = 4
    RED_POLE = 5
    OCTAGON = 6
    BIN_FAR = 7

class MlSrvNode(Node):
    def __init__(self):
        super().__init__("ml_srv")

        self.bridge = CvBridge()
        self.recent_positions: List[Optional[ObjectPosition.Response]] = [None] * len(Targets)
        self.last_image = None

        # Intialize ros services for each of the classes
        for idx, target in enumerate(Targets):
            name = target.name.lower()
            setattr(self, f"{name}_srv",
                    self.create_service(ObjectPosition, f"object_position/{name}",
                                        lambda _, response: self.handle_obj_request(idx, response), # the _ is the request which is unused
                                        ))
            # eg: will define self.gate_red_srv = a service "object_position/gate_red" whose handler is handle_obj_request(0, resp)
            # where 0 is the index of GATE_RED in Targets

        # Subscribe to the ZED left image and depth topics
        # For a full list of zed topics, see https://www.stereolabs.com/docs/ros/zed-node/#published-topics
        self.create_subscription(
            Image,
            "/zed2/zed_node/rgb/image_rect_color",
            self.zed_callback,
            qos_profile=1,
        )
        self.create_subscription(Detections, "/ml/detections", self.detections_callback, qos_profile=1)

        self.run_until_pub = self.create_publisher(Float64, "/ml/run_until", qos_profile=1)
        self.bbox_pub = self.create_publisher(Image, "/ml/annotated", qos_profile=10)

    def zed_callback(self, image: Image):
        self.last_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

    def detections_callback(self, detections: Detections):
        not_found = ObjectPosition.Response()
        not_found.found = False
        new_positions = [not_found] * len(Targets)
        # new_positions is an array containing 1 ObjectPosition.Response() per class

        width = detections.width
        height = detections.height

        bboxs: List[Optional[List[int]]] = [None] * len(Targets)
        # bboxs is a list that contains 1 entry per target class
        # each entry is [x1, y1, x2, y2, confidence] of the detected location of that target class

        # We can assume that most recent detections come from the most recent image
        for d in detections.detections:
            object_pos = ObjectPosition.Response()
            object_pos.found = True

            # camera's field of view in degrees (horizontal and vertical)
            fov_x = 110
            fov_y = 70

            bbox_width = abs(d.right - d.left)
            bbox_height = abs(d.bottom - d.top)
            bbox_area = (bbox_width * bbox_height) / (width * height) # bbox area as a fraction of the image area

            # (x, y) coordinates of bbox center
            x_pos = int((d.left + d.right) / 2)
            y_pos = int((d.top + d.bottom) / 2)

            # (x, y) coordinates of bbox center relative to the center of the ZED image being (0, 0)
            d_x = x_pos - (width / 2)
            d_y = y_pos - (height / 2)

            object_pos.found = True
            object_pos.x_position = x_pos
            object_pos.y_position = y_pos
            object_pos.x_theta = (d_x * fov_x) / width
            object_pos.y_theta = (d_y * fov_y) / height
            object_pos.confidence = d.confidence

            idx: int = d.classification

            box = [int(round(f)) for f in (d.left, d.top, d.right, d.bottom)] + [d.confidence]
            # i.e., box = [(int)x1, (int)y1, (int)x2, (int)y2, (float)confidence]

            new_positions[idx] = object_pos
            bboxs[idx] = box

        for i in range (len(Targets)):
            self.recent_positions[i] = new_positions[i]

        if self.last_image is None:
            return

        image_ocv = np.copy(self.last_image)
        for idx, box in enumerate(bboxs):
            if box is None:
                continue

            # draw a white rectangle of thickness 2 to indicate where the object was detected
            cv2.rectangle(
                image_ocv,
                (box[0], box[1]), # top-left corner
                (box[2], box[3]), # bottom-right corner
                (255, 255, 255),
                2,
            )

            # above the bounding box, put a label containing target class name and confidence
            cv2.putText(
                image_ocv,
                f"{Targets(idx).name} {box[4]:.2f}",
                (int(box[0]), int(box[1] - 5)), # place the text right above the bounding box
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
            )

        msg = self.bridge.cv2_to_imgmsg(image_ocv, encoding="rgb8")
        self.bbox_pub.publish(msg)

    def handle_obj_request(self, idx, response: ObjectPosition.Response):
        # the expression (self.get_clock().now().nanoseconds / 1e9) gives current time in seconds
        # this publisher tells ml_executor to run for next TIME_THRESHOLD seconds
        self.run_until_pub.publish(Float64(data=self.get_clock().now().nanoseconds / 1e9 + TIME_THRESHOLD)) 

        if self.recent_positions[idx] == None:
            response.valid = False
        else:
            response.found = self.recent_positions[idx].found
            response.x_position = self.recent_positions[idx].x_position
            response.y_position = self.recent_positions[idx].y_position
            response.x_theta = self.recent_positions[idx].x_theta
            response.y_theta = self.recent_positions[idx].y_theta
            response.confidence = self.recent_positions[idx].confidence
            response.valid = True

        return response

def main():
    rclpy.init()
    node = MlSrvNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
