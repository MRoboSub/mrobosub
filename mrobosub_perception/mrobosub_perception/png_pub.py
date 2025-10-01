#!/usr/bin/env python

from email.mime import image
import rclpy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from mrobosub_lib import Node

class PngPub(Node):
    def __init__(self):
      
      # Node is publishing to the video_frames topic using
      # the message type Image
      rclpy.init()
      node = rclpy.create_node('png_pub_py')
      node.get_logger().info('Created node')
      self.pub = self.create_publisher(Image,'/zed2/zed_node/rgb/image_rect_color',qos_profile=10)
    
      self.timer = self.create_timer(1 / 10.0, self.loop)

      # Create a VideoCapture object
      # The argument '0' gets the default webcam.
      self.img = cv2.imread("./bbox.png")
      #cap.set(cv2.CAP_PROP_EXPOSURE, -8)

      # Used to convert between ROS and OpenCV images
      self.br = CvBridge()

      #print(type(img))



    def loop(self):
      #While ROS is still running.  
      self.pub.publish(self.br.cv2_to_imgmsg(self.img, encoding='bgr8'))

      
      
def main():
  rclpy.init()
  node = PngPub()
  rclpy.spin(node)
        


if __name__ == "__main__":
  main()