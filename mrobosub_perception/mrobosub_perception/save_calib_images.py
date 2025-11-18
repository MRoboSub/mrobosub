import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading

class ImageSaverNode(Node):
    def __init__(self):
        super.__init__('image_saver_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscriber(Image, '/bot_cam', self.image_callback, qos_profile=1)
        self.image_count = 0
        self.save_directory = 'saved_images'
        self.save_flag = False

        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        # Start save timer
        self.save_timer = self.create_timer(3, self.set_save_flag)

    def image_callback(self, data):
        self.get_logger().info("Received new image")
        try:
            if self.save_flag:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.save_image(cv_image)
                self.save_flag = False
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def set_save_flag(self):
        self.save_flag = True

    def save_image(self, image):
        filename = os.path.join(self.save_directory, f"image_{self.image_count}.jpg")
        cv2.imwrite(filename, image)
        self.get_logger().info(f"Image saved: {filename}")
        self.image_count += 1

def main():
    rclpy.init()
    node = ImageSaverNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()