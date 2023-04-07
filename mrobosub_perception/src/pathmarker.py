#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from mrobosub_lib.lib import Node
from cv_bridge import CvBridge
import cv2
from mrobosub_msgs.srv import PathmarkerAngle, PathmarkerAngleResponse

from pipeline import PathmarkerPipeline

pipeline = PathmarkerPipeline()
resp = PathmarkerAngleResponse()

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
    image = pipeline.hsv_threshold_output

    # cv2.imshow("camera_2", image)
    # cv2.waitKey(1)


def handle_request(msg):
    return resp


if __name__ == "__main__":
    pathmarker_service()
    rospy.spin()