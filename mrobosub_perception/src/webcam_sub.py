#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from pathlib import Path
import time
from datetime import datetime

def callback(data):

  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  # Output debugging information to the terminal
  # rospy.loginfo("receiving video frame")

  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data, desired_encoding='passthrough')

#   # Display image

  if cv2.waitKey(1) == ord(' '):
    Path("./pathmarker_output/").mkdir(exist_ok=True)
    file_name = f"./pathmarker_output/{datetime.today().strftime('%Y-%m-%d-%H:%M:%S')}.png"
    cv2.imwrite(file_name, current_frame)
    last_time = time.time()

  if time.time() - last_time < 5:
    current_frame = cv2.putText(current_frame, "Saved Frame", (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

  cv2.imshow("camera", current_frame)

def receive_message():

  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('bot_sub_py', anonymous=True)

  # Node is subscribing to the video_frames topic
  rospy.Subscriber('bot_cam', Image, callback)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

  # Close down the video stream when done
  cv2.destroyAllWindows()

if __name__ == '__main__':
  last_time = time.time()
  receive_message()