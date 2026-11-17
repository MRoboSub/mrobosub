import rclpy
from rclpy.parameter import Parameter
from mrobosub_lib import Node, Param
from sensor_msgs.msg import Image
import cv2
from mrobosub_perception.cv_bridge_converter import cv2_to_imgmsg, imgmsg_to_cv2

class DummyBotCam(Node):
    def __init__(self):
        super().__init__('dummy_botcam_publisher')

        params = [Param('image_number', Parameter.Type.INTEGER, "image number"), 
                  Param('h', Parameter.Type.INTEGER, "image height"),
                  Param('w', Parameter.Type.INTEGER, "image width")]

        self.declare_params(params)

        self.pub = self.create_publisher( Image, '/dummy_botcam', qos_profile=1)
        self.timer = self.create_timer(1, self.loop)


    def loop(self):
            img_path = f"../dummy_botcam_images/{self.image_number}.png"

            try:
                cv_img = cv2.imread(img_path)
                cv_img = cv2.rectangle(cv_img, (self.w // 2 - 200 - 7, self.h // 2 - 200 + 4), (self.w // 2 + 200 - 7, self.h // 2 + 200 + 4), (255, 255, 255, 3))
                img_msg = cv2_to_imgmsg(cv_img, encoding='bgr8')
                self.pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish dummy botcam image {img_path}: {e}")
                

def main():
    rclpy.init()
    node = DummyBotCam()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
