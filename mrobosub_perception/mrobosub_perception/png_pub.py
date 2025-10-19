
import rclpy
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from mrobosub_lib import Node
import os

class PngPub(Node):
    def __init__(self):
        super().__init__("png_pub")
        # Node is publishing to the video_frames topic using the message type Image
        self.pub = self.create_publisher(Image, '/zed2/zed_node/rgb/image_rect_color', qos_profile=10)
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.

        current_dir = os.path.dirname(os.path.realpath(__file__))
        img_path = os.path.join(current_dir, "bbox.png")
        self.img = cv2.imread(img_path)
        if self.img is None:
            print("Error: image not found!")
        # when I run this, this input is currently "not a numpy array"
        # but we can think about the processing here once we actually have an image we wanna publish

        #cap.set(cv2.CAP_PROP_EXPOSURE, -8)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_message) # Go through the loop 10 times per second

    def publish_message(self):
        self.pub.publish(self.br.cv2_to_imgmsg(self.img, encoding='bgr8'))

def main():
    rclpy.init()
    node = PngPub()
    rclpy.spin(node)

if __name__ == '__main__':
  main()
