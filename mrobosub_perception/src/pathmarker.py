#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node
from cv_bridge import CvBridge
import cv2
from mrobosub_msgs.srv import PathmarkerAngle, PathmarkerAngleResponse
import numpy as np

from pipeline import PathmarkerPipeline

pipeline = PathmarkerPipeline()
resp = PathmarkerAngleResponse()

pub = rospy.Publisher('bot_cam_debug', Image, queue_size=10)
br = CvBridge()

def pathmarker_service():
    rospy.init_node('pathmaker', anonymous=True)
    print("testing")
    rospy.loginfo('inited')
    service = rospy.Service('pathmarker/angle', PathmarkerAngle, handle_request)
    rospy.Subscriber("bot_cam", Image, handle_frame, queue_size=1)
    rospy.loginfo('publishing video frame')


def handle_frame(data):
    bridge = CvBridge()
    image_ocv = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    # Remove the 4th channel (transparency)

    pipeline.process(image_ocv)
    resp.found = pipeline.found
    resp.angle = pipeline.angle
    mask = pipeline.hsv_threshold_output
    image = np.copy(pipeline.soruce)
    image[mask< 255] = 0

    for line in pipeline.filter_lines_output:
        image = cv2.line(image, (line.x1, line.y1), (line.x2, line.y2), (255,0,0), 3)

    pub.publish(br.cv2_to_imgmsg(image))

    # cv2.imshow("camera_2", image)
    # cv2.waitKey(1)


def handle_request(msg):
    return resp


if __name__ == "__main__":
    pathmarker_service()
    rospy.spin()
