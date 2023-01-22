#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float32

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from math import degrees

class StateEstimation(Node):
    """
    Subscribers
    - /depth/raw_depth
    - /imu/data
    """

    """
    Publishers
    - /pose/heave
    - /pose/yaw
    - /pose/pitch
    - /pose/roll
    """
    # pid_params: PIDParams

    heave_offset = 0
    yaw_offset = 0
    pitch_offset = 0
    roll_offset = 0

    orientation = None

    def __init__(self):
        super().__init__('localization')
        self.heave_pub = rospy.Publisher('/pose/heave', Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher('/pose/yaw', Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pose/pitch', Float64, queue_size=1)
        self.roll_pub = rospy.Publisher('/pose/roll', Float64, queue_size=1)    
        rospy.Subscriber('/depth/raw_depth', Float32, self.raw_depth_callback)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)

    def raw_depth_callback(self, raw_depth: Float32):
        if not self.heave_offset:
            self.heave_offset = raw_depth.data
        self.heave_pub.publish(raw_depth.data - self.heave_offset)

    def imu_callback(self, msg):
        orientation = msg.orientation

        quaternion = [
            orientation.x,
            orientation.y, 
            orientation.z, 
            orientation.w
        ]
        euler = euler_from_quaternion(quaternion)
        
        yaw = degrees(-euler[2]) - self.yaw_offset
        pitch = degrees(-euler[1]) - self.pitch_offset
        roll = degrees(euler[0]) - self.roll_offset

        self.yaw_pub.publish(yaw)
        self.pitch_pub.publish(pitch)
        self.roll_pub.publish(roll)

    def run(self):
        rospy.spin()

    def cleanup(self):
        pass

if __name__ == '__main__':
    StateEstimation().run()
