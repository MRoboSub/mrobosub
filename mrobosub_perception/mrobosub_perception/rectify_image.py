#!/usr/bin/env python

import cv2
import numpy as np
from typing import Tuple

from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
from mrobosub_lib import Node

from rcl_interfaces.msg import ParameterDescriptor

def crop_to_circle(image: np.ndarray, radius: int) -> np.ndarray:
    # Find the dimensions of the image
    h, w = image.shape[:2]

    # Create a black mask
    mask = np.zeros((h, w), dtype=np.uint8)

    # Find the center of the image
    cx, cy = w // 2, h // 2

    # Create a white circle
    c_image  = mask # happens in place
    c_center = (cx, cy)
    c_radius = radius
    c_color  = (255, 255, 255)
    c_thickness = -1 # -1 will fill the circle
    cv2.circle(c_image, c_center, c_radius, c_color, thickness=c_thickness)
   
    # bitwise AND the mask onto the original image
    result = cv2.bitwise_and(image, image, mask=mask)
    
    return result


def generate_rectify_maps(h: int, w: int, f: int) -> Tuple[np.ndarray, np.ndarray]:
    # THe theory behind this function is that the image is distorted by a fisheye lens which produces
    # an orthogonal distortion. We need to undistort this raw image to get a rectified image.
    # See https://en.wikipedia.org/wiki/Fisheye_lens.

    # Also this just generates the maps, as the maps just rely on the image dimensions and the
    # f(ocal length), so they can be calculated ahead of time and be reused for every remap.
    # h *= 3
    # w *= 3
    
    # We are going to "oversample" by 3

    cx, cy = w // 2 - 8, h // 2  + 4
   
    # allows us to vectorize our computations
    x_u, y_u = np.meshgrid(np.arange(w), np.arange(h))

    x_rel = x_u - cx
    y_rel = y_u - cy

    #x_rel = x_rel.astype(np.float32)
    #y_rel = y_rel.astype(np.float32)

    #x_rel *= 2
    #y_rel *= 2

    # calculate the distance r_u from the center from the image
    r_u = np.sqrt(x_rel**2 + y_rel**2)
    
    # the angle phi from the verticle
    phi = np.arctan2(y_rel, x_rel)

    # calculate theta from r_u and f
    theta = np.arctan2(r_u, f)

    # Use the formula for the distorted radius for a orthogonal distortion
    r_d = f * np.sin(theta)

    # calculate the corresponding point in the distorted image in the distorted image
    x_d = cx + r_d * np.cos(phi)
    y_d = cy + r_d * np.sin(phi)

    map_x = x_d.astype(np.float32)
    map_y = y_d.astype(np.float32)

    return map_x, map_y

class RectifiedImage(Node):
    def __init__(self):
        super().__init__('rectified_image')

        self.declare_params()

        self.br = CvBridge()
        self.map_x, self.map_y = None, None
        self.shape = None
        
        self.sub = self.create_subscription(Image, '/dummy_botcam', self.handle_frame, qos_profile=1)
        self.sub2 = self.create_subscription(Image, f'/dummy_botcam{self.f}', self.handle_frame, qos_profile=1)
        self.rectified_pub = self.create_publisher(Image, f'/rectified_image', qos_profile=1)

    def handle_frame(self, msg):
        bgr_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.map_x is None or self.map_y is None:
            # Generate the maps only once, as they are static for the given focal length and image size
            self.h, self.w = bgr_img.shape[:2]
            self.map_x, self.map_y = generate_rectify_maps(self.h, self.w, self.f)

        rectified_img = cv2.remap(bgr_img, self.map_x, self.map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        rectified_img = cv2.rectangle(rectified_img, (self.w // 2 - 100 - 7, self.h // 2 - 100 + 4), (self.w // 2 + 100 - 7, self.h // 2 + 100 + 4), (255, 255, 255, 3))
        #rectified_img = cv2.resize(rectified_img, (640, 480), interpolation=cv2.INTER_LINEAR)
        self.rectified_pub.publish(self.br.cv2_to_imgmsg(rectified_img, encoding='bgr8'))


    def declare_params(self):
        param_desc_int = ParameterDescriptor()
        param_desc_int.type = rclpy.Parameter.Type.INTEGER
        param_desc_int.description = "An int parameter"
        self.declare_parameters(
            namespace='',
            parameters = [
                ("f", 280, param_desc_int),
                ("h", 640, param_desc_int),
                ("w", 480, param_desc_int),
            ]
        )
        self.f = self.get_parameter('f').get_parameter_value().integer_value
        self.h = self.get_parameter('h').get_parameter_value().integer_value
        self.w = self.get_parameter('w').get_parameter_value().integer_value


def main():
    rclpy.init()
    node = RectifiedImage()
    rclpy.spin(node)

if __name__== '__main__':
    main()

