#hsv_filter

from typing import Tuple
import cv2
import sys

from cv_bridge import CvBridge
import rclpy
from rclpy import utilities
from rclpy.node import Node
from mrobosub_msgs.srv import ObjectPosition
from mrobosub_perception.timed_service import TimedService
from sensor_msgs.msg import Image

from mrobosub_perception.hsv_pipeline import HsvPipeline
from hsv_pipeline import HsvPipeline

def pixels_to_angles(frame, x_pos: int, y_pos: int, fov_x=110, fov_y=70) -> Tuple[int, int]:
    height, width = frame.shape[0:2]
    d_x = x_pos - (width / 2)
    d_y = y_pos - (height / 2)
    theta_x = (d_x * fov_x) / width
    theta_y = (d_y * fov_y) / height
    return theta_x, theta_y


class BinHsv(Node):
    timing_threshold: float

    def __init__(self, always_run: bool):
        super().__init__('bin_hsv')

        self.declare_params()
        self.timing_threshold = self.get_parameter('timing_threshold').get_parameter_value().double_value

        self.br = CvBridge()

        self.always_run = always_run

        self.sub = self.create_subscription(Image, '/rectified_image', self.handle_frame, qos_profile=1)
        self.serv = TimedService(self, '/bin_object_position', ObjectPosition, self.timing_threshold)
        self.mask_pub = self.create_publisher(Image, f'/bin_mask', qos_profile=1)
        self.enhanced_pub = self.create_publisher(Image, f'/bin_enhanced', qos_profile=1)
        self.annotated_pub = self.create_publisher(Image, f'/bin_annotated', qos_profile=1)

    def handle_frame(self, msg):
        if(self.serv.should_run() or self.always_run):
            bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pipeline = HsvPipeline(**self.hsv_params, color_space=cv2.COLOR_BGR2HSV)
            mask, enhanced_img = pipeline.filter_image(bgr_img, return_enhanced=True)
            detection = pipeline.find_circular_object(mask)

            if detection is not None:
                annotated_img = cv2.drawMarker(bgr_img, (detection.x,detection.y), (255,255,255), markerType=cv2.MARKER_CROSS)
            else:
                annotated_img = bgr_img

            self.mask_pub.publish(self.br.cv2_to_imgmsg(mask, encoding='mono8'))
            self.enhanced_pub.publish(self.br.cv2_to_imgmsg(enhanced_img, encoding='bgr8'))
            self.annotated_pub.publish(self.br.cv2_to_imgmsg(annotated_img, encoding='bgr8'))
            
            response = ObjectPositionResponse()
            if detection is not None:
                x_theta, y_theta = pixels_to_angles(bgr_img, detection.x, detection.y)
                response.found = True
                response.x_position = detection.x / bgr_img.shape[1]
                response.y_position = detection.y / bgr_img.shape[0]
                response.x_theta = x_theta
                response.y_theta = y_theta

            self.serv.set_result(response)

    def declare_params(self):
        self.declare_parameter(
            "timing_threshold", 0.0
        )
        
    
def main():
    rclpy.init()
    node = BinHsv(rclpy.utilities.remove_ros_args(sys.argv)[1] != "0")
    rclpy.spin(node)


if __name__=='__main__' :
    main()
