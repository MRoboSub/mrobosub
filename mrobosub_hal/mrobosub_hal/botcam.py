import rclpy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import subprocess
import numpy as np

from mrobosub_lib import Node, Param
from rclpy.parameter import Parameter

from std_srvs.srv import SetBool

class Botcam(Node):
    """
    Services:
    /bot_cam/on 

    Publishes: 
    /bot_cam
    /rectified_image
    """

    def __init__(self) -> None:
        super().__init__("bot_cam")

        # Set up focal length as a parameter that can be dynamically changed and updated.
        params = [Param('f', Parameter.Type.INTEGER, "Focal length of the camera.", self.f_callback)]
        self.declare_params(params)

        # Camera settings
        self.device_path = "/dev/botcam"
        self.w = 960
        self.h = 540
        self.output_w = int(self.w / 2)
        self.output_h = int(self.h / 2)

        # Create service to turn on the Camera.
        self.on = False
        self.create_service(SetBool, "/bot_cam/on", self.handle_on_service)

        # Create publishers        
        self.pub = self.create_publisher(Image, "/bot_cam", qos_profile=1)
        self.rectified_pub = self.create_publisher(Image, "/rectified_image", qos_profile=1)

        # Create OpenCV Objects
        self.br = CvBridge()

        # Apply undistortion for fish eye lens 
        self.map_x, self.map_y = self.generate_undistort_maps(self.f, self.w, self.h)

        # Run loop
        self.iteration_rate = 60
        self.timer = self.create_timer(1.0/self.iteration_rate, self.loop)


    def handle_on_service(self, req, res):
        res.success = True
        if req.data == self.on:
            return res

        if req.data:
            self.open_capture()
        else:
            self.close_capture()
        self.on = req.data
        return res

    def open_capture(self):
        self.cap = cv2.VideoCapture(self.device_path)
        # https://stackoverflow.com/a/66279297
        # cap.set(cv2.CAP_PROP_FPS,10)
        # cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        # self.cap.set(6, 1296718151) # wtf
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 1000)
        subprocess.call(
            f"v4l2-ctl -d {self.device_path} -c white_balance_temperature_auto=0 -c brightness=-64 -c exposure_auto=1 -c exposure_absolute=4 -c contrast=50",
            shell=True,
        )

    def close_capture(self):
        self.cap.release()

    def loop(self):
        if not self.on:
            return

        success, frame = self.cap.read()

        if success:
            # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            rectified = self.undistort(frame)
            self.pub.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
            resized = cv2.resize(rectified, (self.output_w, self.output_h))
            self.rectified_pub.publish(self.br.cv2_to_imgmsg(resized, encoding="bgr8"))

    def undistort(self, bgr_img):
        rectified_img = cv2.remap(bgr_img, self.map_x, self.map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # rectified_img = cv2.rectangle(rectified_img, (self.w // 2 - 100 - 7, self.h // 2 - 100 + 4), (self.w // 2 + 100 - 7, self.h // 2 + 100 + 4), (255, 255, 255, 3))
        # rectified_img = cv2.resize(rectified_img, (640, 480), interpolation=cv2.INTER_LINEAR)
        return rectified_img

    def f_callback(self, new_f_value):
        self.map_x, self.map_y = self.generate_undistort_maps(new_f_value, self.w, self.h)
    

    def generate_undistort_maps(self, f, w, h):
        # The theory behind this function is that the image is distorted by a fisheye lens which produces
        # an orthogonal distortion. We need to undistort this raw image to get a rectified image.
        # See https://en.wikipedia.org/wiki/Fisheye_lens.
        cx, cy = w // 2 - 8, h // 2  + 4

        # allows us to vectorize our computations
        x_u, y_u = np.meshgrid(np.arange(w), np.arange(h))

        x_rel = x_u - cx
        y_rel = y_u - cy
        x_rel *= 2
        y_rel *= 2

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

def main():
    rclpy.init()
    node = Botcam()
    rclpy.spin(node)  

if __name__ == "__main__":
    main()
