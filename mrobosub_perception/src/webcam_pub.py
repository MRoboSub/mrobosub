#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

def publish_message():

  # Node is publishing to the video_frames topic using
  # the message type Image
  pub = rospy.Publisher('bot_cam', Image, queue_size=1)

  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('bot_pub_py', anonymous=True)

  # Go through the loop 30 times per second
  rate = rospy.Rate(30) # 30hz

  # Create a VideoCapture object
  cap = cv2.VideoCapture("/dev/video10")
  # https://stackoverflow.com/a/66279297
  cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
  cap.set(cv2.CAP_PROP_EXPOSURE, 100)
  #cap.set(cv2.CAP_PROP_FPS,30)
  #cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  # While ROS is still running.
  while not rospy.is_shutdown():

      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret, frame = cap.read()

      if ret == True:
        # Print debugging information to the terminal
        #rospy.loginfo('publishing video frame')

        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        pub.publish(br.cv2_to_imgmsg(frame, encoding='bgr8'))

      # Sleep just enough to maintain the desired rate
      rate.sleep()

if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException as e:
    raise e
