import rclpy
from mrobosub_lib.lib import Node
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from mrobosub_perception.cfg import dummy_botcam_paramsConfig
from cv_bridge import CvBridge
import cv2

class DummyBotCam(Node):
    def __init__(self):
        super().__init__('dummy_botcam')
        self.br = CvBridge()
        self.pub = self.create.publisher( Image, '/dummy_botcam', qos_profile=1)
        self.declare_parameter("image_number", 1)
        self.srv = Server(dummy_botcam_paramsConfig, self.reconfigure_callback, 'dummy_botcam_config')
        self.image_number = self.get_parameter('image_number').get_parameter_value().int_value
        self.timer = self.create_timer(1, self.loop)
        

    def loop(self):
            img_path = f"../dummy_botcam_images/{self.image_number}.png"
            try:
                cv_img = cv2.imread(img_path)
                self.w = 480
                self.h = 640
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
