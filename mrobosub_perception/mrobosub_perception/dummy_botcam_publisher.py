import rclpy
from mrobosub_lib import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DummyBotCam(Node):
    def __init__(self):
        super().__init__('dummy_botcam_publisher')
        self.br = CvBridge()
        self.declare_params()
        self.add_post_set_parameters_callback(self.set_params)
        self.pub = self.create_publisher( Image, '/dummy_botcam', qos_profile=1)
        self.timer = self.create_timer(1, self.loop)
    
    def set_params(self, _params = None):
        self.image_number = self.get_parameter('image_number').get_parameter_value().integer_value
        self.h = self.get_parameter('h').get_parameter_value().integer_value
        self.w = self.get_parameter('w').get_parameter_value().integer_value

    def declare_params(self):
        self.declare_parameter("image_number", 1)
        self.declare_parameter("h", 0)
        self.declare_parameter("w", 0)
        self.set_params()

    def loop(self):
            img_path = f"../dummy_botcam_images/{self.image_number}.png"

            try:
                cv_img = cv2.imread(img_path)
                cv_img = cv2.rectangle(cv_img, (self.w // 2 - 200 - 7, self.h // 2 - 200 + 4), (self.w // 2 + 200 - 7, self.h // 2 + 200 + 4), (255, 255, 255, 3))
                img_msg = self.br.cv2_to_imgmsg(cv_img, encoding='bgr8')
                self.pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish dummy botcam image {img_path}: {e}")
                

def main():
    rclpy.init()
    node = DummyBotCam()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
