import cv2
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge

from mrobosub_lib.lib import ControlLoopNode, Param




class Zed(ControlLoopNode):
    device_path = Param[str]

    def __init__(self):
        super().__init__('zed')
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(self.device_path)
        self.pub = rospy.Publisher(
            '/zed2/zed_node/rgb/image_rect_color', 
            Image, queue_size=1
        )

    def chop(self, frame):
        width = len(frame[0])
        return frame[:(width//2)]

    def loop(self):
        success, frame = self.cap.read()

        if success:
            frame = self.chop(frame)
            self.pub.publish(self.br.cv2_to_imgmsg(frame))

            

if __name__ == '__main__':
    Zed().run()
