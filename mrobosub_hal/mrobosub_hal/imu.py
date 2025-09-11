#!/usr/bin/env python

import inertial_sense_ros.msg as imu_msgs
import rclpy

from mrobosub_lib.lib import Node
from mrobosub_msgs.msg import Imu_INS, Imu_PIMU

class IMU(Node):
    '''
    This node reads did_ins and pimu values from the /ins_eul_uvw_ned and /pimu topics respectively,
    which are published to by the third-party inertial_sense_ros code. Each time there's a fresh value
    published to these topics, it unpacks the value into objects of our custom data types (Imu_INS and Imu_PIMU)
    and publishes this to /imu_INS or /imu_PIMU respectively, from where it is read by downstream nodes.
    '''
    def __init__(self):
        super().__init__('imu')
        self.get_logger().info("Launched imu node")
        self.did_ins_sub = self.create_subscription(imu_msgs.DID_INS1, "/ins_eul_uvw_ned", self.did_ins_callback, 1) # for DID_INS1
        self.pimu_sub = self.create_subscription(imu_msgs.PIMU, "/pimu", self.did_pimu_callback, 1) # for DID_PIMU
        self.did_ins_pub = self.create_publisher(Imu_INS, "/imu_INS", qos_profile=1)
        self.pimu_pub = self.create_publisher(Imu_PIMU, "/imu_PIMU", qos_profile=1) 

    def did_ins_callback(self, msg):
        (x, y, z) = msg.theta
        self.did_ins_pub.publish((x, y, -z))

    def did_pimu_callback(self, msg):
        m = Imu_PIMU()
        m.dtheta = msg.dtheta
        m.dvel = msg.dvel
        m.dt = msg.dt

        self.pimu_pub.publish(m)


def main():
    rclpy.init()
    node = IMU()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
