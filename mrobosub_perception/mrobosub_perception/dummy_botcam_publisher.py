import rospy
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
        self.pub = rospy.Publisher('/dummy_botcam', Image, queue_size=1)
        self.srv = Server(dummy_botcam_paramsConfig, self.reconfigure_callback, 'dummy_botcam_config')
        self.image_number = 1

    def reconfigure_callback(self, config, level):
        self.image_number = config["image_number"]
        return config

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            img_path = f"../dummy_botcam_images/{self.image_number}.png"
            try:
                cv_img = cv2.imread(img_path)
                self.w = 480
                self.h = 640
                cv_img = cv2.rectangle(cv_img, (self.w // 2 - 200 - 7, self.h // 2 - 200 + 4), (self.w // 2 + 200 - 7, self.h // 2 + 200 + 4), (255, 255, 255, 3))
                img_msg = self.br.cv2_to_imgmsg(cv_img, encoding='bgr8')
                self.pub.publish(img_msg)
            except Exception as e:
                rospy.logerr(f"Failed to publish dummy botcam image {img_path}: {e}")

if __name__ == "__main__":
    node = DummyBotCam()
    node.run()

