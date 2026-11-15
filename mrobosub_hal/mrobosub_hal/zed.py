import cv2
import numpy as np
import array
import math
import time
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import rclpy
import sys

from mrobosub_lib import Node

import pyzed.sl as sl # type: ignore

from std_srvs.srv import SetBool
from mrobosub_hal.hole_detection import HoleDetector, HoleTracker, Intrinsics

# encoding -> (numpy dtype, channels)
ENCODINGS = {
    "bgr8": (np.uint8, 3),
    "32FC1": (np.float32, 1),
}

def to_imgmsg(arr, encoding):
    """Convert a numpy image to sensor_msgs/Image without cv_bridge.

    Uses array.array for msg.data: assigning raw bytes makes the ROS 2 message
    class validate every byte in Python, which is very slow for full-size images.
    """
    dtype, channels = ENCODINGS[encoding]
    arr = np.ascontiguousarray(arr, dtype=dtype)
    h, w = arr.shape[:2]
    msg = Image()
    msg.height, msg.width = h, w
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = w * channels * arr.itemsize
    data = array.array("B")
    data.frombytes(arr.tobytes())
    msg.data = data
    return msg

class Zed(Node):
    """
    Provides /zed/on service and /zed/raw topic and /zed2/zed_node/rgb/image_rect_color topic
    """

    def __init__(self):
        super().__init__("zed")
        self.iteration_rate = 60
        self.device_path = sys.argv[1]
        self.on = False
        self.zed = sl.Camera()
        self.image = sl.Mat()
        self.depth = sl.Mat() 
        self.depth_small = sl.Mat()
        self.sensors_data = sl.SensorsData()
                          
        self.raw_pub = self.create_publisher(Image, "/zed/raw", qos_profile=1)
        self.pub = self.create_publisher(Image, "/zed2/zed_node/rgb/image_rect_color", qos_profile=1)
        self.depth_pub = self.create_publisher(Image, "/zed2/zed_node/depth", qos_profile=1)
        self.debug_pub = self.create_publisher(Image, "/zed/holes_debug", qos_profile=1)
        self.holes_pub = self.create_publisher(PoseArray, "/zed/holes", qos_profile=1)
        self.imu_pub = self.create_publisher(Imu, "/zed/imu", qos_profile=10)
        self.odom_pub = self.create_publisher(Odometry, "/zed/odom", qos_profile=10)
        self.zed_pose = sl.Pose()
        
        self.create_service(SetBool, "/zed/on", self.handle_on_service)
        self.timer = self.create_timer(1.0/self.iteration_rate, self.loop)

    def handle_on_service(self, req, res):
        self.get_logger().info("Zed on request")
        res.success = True
        if req.data == self.on:
            # nothing to be done, camera is already in the desired state
            return res

        if req.data:
            self.get_logger().info("Zed turning on")
            self.open_capture()
        else:
            self.get_logger().info("Zed turning off")
            self.close_capture()
        self.on = req.data

        self.get_logger().info("Service handeled")
        return res

    def open_capture(self):
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720    # Use HD720 video mode for USB cameras
        init_params.camera_fps = 60
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(repr(err))
            exit(-1)
        else:
            self.get_logger().info("Zed intialized")
    
        self.runtime_param = sl.RuntimeParameters()
        cfg = self.zed.get_camera_information().camera_configuration
        cal = cfg.calibration_parameters.left_cam
        s = 0.5  # hole detection runs at half resolution
        w, h = int(cfg.resolution.width * s), int(cfg.resolution.height * s)
        self.det_scale = s
        self.det_res = sl.Resolution(w, h)
        self.detector = HoleDetector(
            Intrinsics(cal.fx * s, cal.fy * s, cal.cx * s, cal.cy * s), (h, w),
            min_circularity=0.6, min_diameter_m=0.02, max_diameter_m=2.0,
            edge_erode_px=8,
        )
        self.tracker = HoleTracker()

        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1  # SDK 5 default (GEN 3) fails to start here
        err = self.zed.enable_positional_tracking(tracking_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Positional tracking failed to start: {err!r}")

    def close_capture(self):
        self.zed.close()

    def chop(self, frame):
        width = frame.shape[1]
        return frame[:, : (width // 2), :]

    def crop(self, frame):
        left, right, top, bottom = 130, 50, 40, 60
        frame[:, :left] = frame[:, -right:] = frame[:top, :] = frame[-bottom:, :] = [
            255,
            0,
            0,
        ]
        return frame

    def publish_imu(self, stamp):
        if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.IMAGE) != sl.ERROR_CODE.SUCCESS:
            return
        d = self.sensors_data.get_imu_data()
        msg = Imu()
        msg.header = Header(stamp=stamp, frame_id="zed_imu_link")
        ox, oy, oz, ow = d.get_pose().get_orientation().get()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, (ox, oy, oz, ow))
        gx, gy, gz = d.get_angular_velocity()          # SDK gives deg/s
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (math.radians(float(g)) for g in (gx, gy, gz))
        ax, ay, az = d.get_linear_acceleration()       # m/s^2
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, (ax, ay, az))
        self.imu_pub.publish(msg)

    def publish_odom(self, stamp):
        state = self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
        self.get_logger().info(f"tracking state: {state}", throttle_duration_sec=1.0)  # DEBUG
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            return                                     # dropouts show up as gaps, not bad data
        msg = Odometry()
        msg.header = Header(stamp=stamp, frame_id="zed_odom")
        msg.child_frame_id = "zed_camera_link"
        tx, ty, tz = self.zed_pose.get_translation().get()
        ox, oy, oz, ow = self.zed_pose.get_orientation().get()
        p = msg.pose.pose
        p.position.x, p.position.y, p.position.z = map(float, (tx, ty, tz))
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (ox, oy, oz, ow))
        cov = np.asarray(self.zed_pose.pose_covariance, dtype=float).ravel()
        if cov.size == 36:
            msg.pose.covariance = cov.tolist()
        self.odom_pub.publish(msg)

    def publish_holes(self, holes, stamp):
        msg = PoseArray()
        msg.header = Header(stamp=stamp, frame_id="zed_left_camera_optical")   # X right, Y down, Z forward
        for h in holes:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, h.center_xyz)
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.holes_pub.publish(msg)                    # empty array = no holes seen

    def loop(self):
        if not self.on:
            return
        
        t0 = time.perf_counter()  # DEBUG
        if self.zed.grab(self.runtime_param) != sl.ERROR_CODE.SUCCESS:
            return

        stamp = self.get_clock().now().to_msg()
                
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        frame = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)
        self.raw_pub.publish(to_imgmsg(frame, "bgr8"))
        self.pub.publish(to_imgmsg(self.crop(frame.copy()), "bgr8"))
        
        self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
        self.depth_pub.publish(to_imgmsg(self.depth.get_data(), "32FC1"))
        
        self.publish_imu(stamp)
        self.publish_odom(stamp)
        
        t1 = time.perf_counter()  # DEBUG
        self.zed.retrieve_measure(self.depth_small, sl.MEASURE.DEPTH, sl.MEM.CPU, self.det_res)
        holes = self.tracker.update(self.detector.detect(self.depth_small.get_data()))
        self.publish_holes(holes, stamp)
        if self.debug_pub.get_subscription_count() > 0:  # only build the debug image when someone is watching
            debug = frame.copy()
            for h in holes:
                u, v = (int(c / self.det_scale) for c in h.center_px)
                cv2.circle(debug, (u, v), int(h.radius_px / self.det_scale), (0, 255, 0), 2)
                cv2.putText(debug, f"{h.center_xyz[2]:.2f}m", (u + 5, v), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            self.debug_pub.publish(to_imgmsg(debug, "bgr8"))
        t2 = time.perf_counter()  # DEBUG
        self.get_logger().info(  # DEBUG
            f"loop {1000*(t2-t0):.0f} ms | grab+images+depth+imu+odom {1000*(t1-t0):.0f} ms | holes {1000*(t2-t1):.0f} ms",
            throttle_duration_sec=1.0,
        )
        
def main():
    rclpy.init()
    node = Zed()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

