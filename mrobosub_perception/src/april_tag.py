#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from apriltag import apriltag
import numpy as np

class AprilTagDetector:
    def __init__(self):
        rospy.init_node('april_tag_detector', anonymous=True)
        self.bridge = CvBridge()
        self.detector = apriltag.Detector("tagCustom48h12", threads=1)
        self.image_pub = rospy.Publisher("/tag_detected_image", Image, queue_size=1)
        self.april_tag_pub = rospy.Publisher("/mrobosub_msgs/april_tag_detection", AprilTagDetector, queue_size=1)
        self.camera_sub = rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, self.handle_frame, queue_size=1)
        #self.camera_sub = rospy.Subscriber("bot_cam", Image, self.handle_frame, queue_size=1)
        self.tag_size = 81  # in millimeter #TODO: Measure TAG SIZE
        self.small_tag_size = 10.8  # in millimeter
        self.object_points = np.array([
            [-self.tag_size / 2, self.tag_size / 2, 0],  # Top-left corner
            [self.tag_size / 2, self.tag_size / 2, 0],  # Top-right corner
            [self.tag_size / 2, -self.tag_size / 2, 0],  # Bottom-right corner
            [-self.tag_size / 2, -self.tag_size / 2, 0],  # Bottom-left corner
        ], dtype=np.float32)
        self.small_object_points = np.array([
            [-self.small_tag_size / 2, self.small_tag_size / 2, 0],  # Top-left corner
            [self.small_tag_size / 2, self.small_tag_size / 2, 0],  # Top-right corner
            [self.small_tag_size / 2, -self.small_tag_size / 2, 0],  # Bottom-right corner
            [-self.small_tag_size / 2, -self.small_tag_size / 2, 0],  # Bottom-left corner
        ], dtype=np.float32)
        self.intrinsic_matrix = np.array([449.0419, 0, 0], [0, 447.5708, 0], [350.6880, 237.6657, 1.0])

    
    def handle_frame(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        if detections:
            for detect in detections:
                # Extract corners from the detection result
                corners = detect.corners.astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(cv_image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

                if detect.tag_id < 10:  # big tag
                    image_points = detect.corners.astype(np.float32)
                    retval, rvec, tvec = cv2.solvePnP(self.object_points, image_points, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if detect.tag_id >= 10:  # small tag at center
                    image_points = detect.corners.astype(np.float32)
                    retval, rvec, tvec = cv2.solvePnP(self.small_object_points, image_points, self.intrinsic_matrix, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                rotation_matrix, _ = cv2.Rodrigues(rvec)
                roll, pitch, yaw = self.calculate_euler_angles_from_rotation_matrix(rotation_matrix)

                pos_text = f"Tag ID {detect.tag_id}: x={tvec[0][0]:.2f}, y={tvec[1][0]:.2f}, z={tvec[2][0]:.2f},"
                orientation_text = f" roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}"
                vertical_pos = 40  # Adjust this value as needed for positioning text in the image
                cv2.putText(cv_image, pos_text + orientation_text, (10, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 0, 0), 2)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


    def calculate_euler_angles_from_rotation_matrix(self, R):
        """Calculate Euler angles (roll, pitch, yaw) from a rotation matrix.
        Assumes the rotation matrix uses the XYZ convention.
        """
        sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2,1], R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0

        return np.rad2deg(x), np.rad2deg(y), np.rad2deg(z)  # Convert to degrees


if __name__ == '__main__':
    try:
        detector = AprilTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
