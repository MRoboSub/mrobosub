#!/usr/bin/env python

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

def publish_message():

  # Node is publishing to the video_frames topic using
  # the message type Image
  pub = rospy.Publisher('zed2/zed_node/rgb/image_rect_color', Image, queue_size=10)

  rospy.init_node('png_pub_py', anonymous=True)

  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz

  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  img = cv2.imread("./test_ml.png")
  #cap.set(cv2.CAP_PROP_EXPOSURE, -8)

  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  # While ROS is still running.
  while not rospy.is_shutdown():
    pub.publish(br.cv2_to_imgmsg(img, encoding='bgr8'))

    # Sleep just enough to maintain the desired rate
    rate.sleep()

if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException as e:
    raise e