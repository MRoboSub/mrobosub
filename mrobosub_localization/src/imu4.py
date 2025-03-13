#!/usr/bin/env python

import inertial_sense_ros.msg
import rospy

from mrobosub_lib.lib import Node
from std_msgs.msg import Float32
import inertial_sense_ros

NUM_MOTORS = 8

class IMU(Node):
    def __init__(self):
        super().__init__('imu4')
        print("Launched imu4 node")
        self.imu_sub = rospy.Subscriber("/did_ins1", 10, self.imu_callback)
        rospy.spin()

    def imu_callback(self):
        print("got msg!!")

def main():
    IMU()

if __name__ == "__main__":
    main()