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
from mrobosub_msgs.msg import SegDetections
from mrobosub_msgs.srv import SegObject

CONFIDENCE = 0.5
TIME_THRESHOLD = 10


class SegTargets(Enum):
    PATHMARKER = 0
    SANDBAG = 1


class SegSrvNode(Node):
    def __init__(self):
        super().__init__("seg_srv")

        self.bridge = CvBridge()
        self.recent_positions: List[Optional[SegObject.Response]] = [None] * len(SegTargets)
        self.last_image = None

        # Initialize ros services for each of the classes
        for idx, target in enumerate(SegTargets):
            name = target.name.lower()
            setattr(self, f"{name}_srv",
                    self.create_service(SegObject, f"seg_object/{name}",
                                        # idx=idx binds the CURRENT loop value into the lambda's default args;
                                        # without it every handler would close over the same `idx` variable and
                                        # all services would end up reporting on the last target (BIN_FAR).
                                        lambda _, response, idx=idx: self.handle_object_request(idx, response),
                                        ))
            # eg: will define self.gate_back_srv = a service "seg_object/gate_back" whose handler is
            # handle_object_request(4, resp), where 4 is the index of GATE_BACK in SegTargets

        # Subscribe to the rectified image (for annotated visualization) and the seg model's detections
        self.create_subscription(
            Image,
            "/rectified_image",
            self.image_callback,
            qos_profile=1,
        )
        self.create_subscription(SegDetections, "/seg/detections", self.detections_callback, qos_profile=1)

        self.run_until_pub = self.create_publisher(Float64, "/seg/run_until", qos_profile=1)
        self.mask_pub = self.create_publisher(Image, "/seg/annotated", qos_profile=10)

    def image_callback(self, image: Image):
        self.last_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

    def detections_callback(self, detections: SegDetections):
        not_found = SegObject.Response()
        not_found.found = False
        new_positions = [not_found] * len(SegTargets)
        # new_positions is an array containing 1 SegObject.Response() per class

        width = detections.width
        height = detections.height

        # best_area/polygons_px track, per class, the largest-area instance seen so far this frame;
        # any smaller instance of the same class is ignored, per spec
        best_area = [-1.0] * len(SegTargets)
        polygons_px: List[Optional[np.ndarray]] = [None] * len(SegTargets)

        for d in detections.detections:
            if d.confidence < CONFIDENCE:
                continue

            idx: int = d.classification
            polygon = np.array([[p.x, p.y] for p in d.polygon], dtype=np.float32)
            if polygon.shape[0] < 3:
                continue  # degenerate polygon, not enough points to form a shape

            area_px = cv2.contourArea(polygon)
            if area_px <= best_area[idx]:
                continue  # a bigger instance of this class was already seen this frame

            best_area[idx] = area_px
            polygons_px[idx] = polygon

            object_pos = SegObject.Response()
            object_pos.found = True

            # size: % of image area covered by the mask
            object_pos.size = 100 * area_px / (width * height)

            # centroid of the mask polygon
            moments = cv2.moments(polygon)
            if moments["m00"] != 0:
                cx = moments["m10"] / moments["m00"]
                cy = moments["m01"] / moments["m00"]
            else:
                cx, cy = float(polygon[:, 0].mean()), float(polygon[:, 1].mean())
            object_pos.x_position = float(cx)
            object_pos.y_position = float(cy)

            # direction: angle of the mask's long axis, normalized to (-90, 90]
            # (a line's orientation only has meaning mod 180 degrees, so there's no "front"/"back" here,
            # just tilt left vs. tilt right)
            (_, _), (rect_w, rect_h), angle = cv2.minAreaRect(polygon)
            if rect_w < rect_h:
                angle += 90
            if angle > 90:
                angle -= 180
            object_pos.direction = float(angle)

            object_pos.confidence = float(d.confidence)

            new_positions[idx] = object_pos

        for i in range(len(SegTargets)):
            self.recent_positions[i] = new_positions[i]

        if self.last_image is None:
            return

        image_ocv = np.copy(self.last_image)
        for idx, polygon in enumerate(polygons_px):
            if polygon is None:
                continue

            # draw the actual mask outline (not a bbox) to indicate where the object was detected
            pts = polygon.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(image_ocv, [pts], isClosed=True, color=(255, 255, 255), thickness=2)

            pos = new_positions[idx]
            cv2.putText(
                image_ocv,
                f"{SegTargets(idx).name} {pos.confidence:.2f} {pos.direction:.0f}deg",
                (int(pos.x_position), int(pos.y_position - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
            )

        msg = self.bridge.cv2_to_imgmsg(image_ocv, encoding="bgr8")
        self.mask_pub.publish(msg)

    def handle_object_request(self, idx, response: SegObject.Response):
        # this publisher tells seg_executor to run for the next TIME_THRESHOLD seconds
        self.run_until_pub.publish(Float64(data=self.get_clock().now().nanoseconds / 1e9 + TIME_THRESHOLD))

        if self.recent_positions[idx] is None:
            response.valid = False
        else:
            response.found = self.recent_positions[idx].found
            response.x_position = self.recent_positions[idx].x_position
            response.y_position = self.recent_positions[idx].y_position
            response.size = self.recent_positions[idx].size
            response.direction = self.recent_positions[idx].direction
            response.confidence = self.recent_positions[idx].confidence
            response.valid = True

        return response


def main():
    rclpy.init()
    node = SegSrvNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
