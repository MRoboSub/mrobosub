#!/usr/bin/env python

from typing import List, Optional
from enum import Enum

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from mrobosub_lib.lib import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from mrobosub_msgs.msg import Detections
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse


CONFIDENCE = 0.9
TIME_THRESHOLD = 10


class Targets(Enum):
    GATE_RED = 0
    GATE_BLUE = 1


class MlSrvNode(Node):
    recent_positions: List[Optional[ObjectPositionResponse]]
    red_is_left: bool = True

    def __init__(self):
        super().__init__("ml_srv")

        self.bridge = CvBridge()

        # Intialize ros services for each of the objects
        mk_service = lambda name, idx: rospy.Service(
            f"object_position/{name}",
            ObjectPosition,
            lambda msg: self.handle_obj_request(idx.value, msg),
        )
        self.gate_red_srv = mk_service("gate_red", Targets.GATE_RED)
        self.gate_blue_srv = mk_service("gate_blue", Targets.GATE_BLUE)

        self.recent_positions = [None] * len(Targets)

        # Subscribe to the ZED left image and depth topics
        # For a full list of zed topics, see https://www.stereolabs.com/docs/ros/zed-node/#published-topics
        rospy.Subscriber(
            "/zed2/zed_node/rgb/image_rect_color",
            Image,
            self.zed_callback,
            queue_size=1,
        )
        rospy.Subscriber("/ml/detections", Detections, self.detections_callback)

        self.run_until_pub = rospy.Publisher("/ml/run_until", Float64, queue_size=1)

        self.bbox_pub = rospy.Publisher("/ml/annotated", Image, queue_size=10)
        self.last_image = None

    def zed_callback(self, image: Image):
        self.last_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")

    def detections_callback(self, detections: Detections):
        not_found = ObjectPositionResponse()
        not_found.found = False
        new_positions = [not_found] * len(Targets)

        width = detections.width
        height = detections.height

        bboxs: List[Optional[List[int]]] = [None] * len(Targets)

        # We can assume that most recent detections come from the most recent image
        for d in detections.detections:
            object_pos = ObjectPositionResponse()
            object_pos.found = True

            fov_x = 110
            fov_y = 70

            bbox_width = abs(d.right - d.left)
            bbox_height = abs(d.bottom - d.top)
            bbox_area = (bbox_width * bbox_height) / (width * height)

            x_pos = int((d.left + d.right) / 2)
            y_pos = int((d.top + d.bottom) / 2)

            d_x = x_pos - (width / 2)
            d_y = y_pos - (height / 2)

            theta_x = (d_x * fov_x) / width
            theta_y = (d_y * fov_y) / height

            object_pos.found = True
            # object_pos.x_position = x_pos
            object_pos.x_position = bbox_area  # TODO: change this. for competition
            object_pos.y_position = y_pos
            object_pos.x_theta = theta_x
            object_pos.y_theta = theta_y
            object_pos.confidence = d.confidence

            idx = int(d.classification)

            if idx < 2:  # for red and blue gate symbols
                if new_positions[0] and new_positions[0].found:
                    if new_positions[0].x_theta < object_pos.x_theta:
                        if self.red_is_left:
                            idx = 1
                        else:
                            idx = 0
                    else:
                        if self.red_is_left:
                            idx = 0
                        else:
                            idx = 1
                elif new_positions[1] and new_positions[1].found:
                    if new_positions[1].x_theta < object_pos.x_theta:
                        if self.red_is_left:
                            idx = 1
                        else:
                            idx = 0
                    else:
                        if self.red_is_left:
                            idx = 0
                        else:
                            idx = 1

            box = [int(round(f)) for f in (d.left, d.top, d.right, d.bottom)] + [
                d.confidence
            ]

            if new_positions[idx] and new_positions[idx].found:
                new_positions[1 - idx] = new_positions[idx]
                bboxs[1 - idx] = bboxs[idx]

            new_positions[idx] = object_pos
            bboxs[idx] = box

        self.recent_positions = new_positions

        if self.last_image is None:
            return
        image_ocv = np.copy(self.last_image)
        for idx, box in enumerate(bboxs):
            if box is None:
                continue
            cv2.rectangle(
                image_ocv,
                (box[0], box[1]),
                (box[2], box[3]),
                (255, 255, 255),
                2,
            )
            cv2.putText(
                image_ocv,
                f"{Targets(idx).name} {box[4]:.2f}",
                (int(box[0]), int(box[1] - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
            )

        msg = self.bridge.cv2_to_imgmsg(image_ocv, encoding="rgb8")
        self.bbox_pub.publish(msg)

    def handle_obj_request(self, idx, msg):
        self.run_until_pub.publish(rospy.get_time() + TIME_THRESHOLD)
        while self.recent_positions[idx] == None:
            rospy.sleep(0.005)
        return self.recent_positions[idx]

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    MlSrvNode().run()
