#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float32

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

from std_srvs.srv import Trigger
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from iekf import IEKF

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

    heave_offset = None
    yaw_offset = None
    pitch_offset = None
    roll_offset = None

    orientation = None

    def __init__(self):
        self.iekf_class = IEKF()
        super().__init__('localization')
        self.heave_pub = rospy.Publisher('/pose/heave', Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher('/pose/yaw', Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pose/pitch', Float64, queue_size=1)
        self.roll_pub = rospy.Publisher('/pose/roll', Float64, queue_size=1) 
        self.x_pub = rospy.Publisher('/pose/x_pos', Float64, queue_size=1)
        self.y_pub = rospy.Publisher('/pose/y_pos', Float64, queue_size=1)
        rospy.Subscriber('/dvl/raw_data', Dvl, self.dvl_callback)   
        rospy.Subscriber('/depth/raw_depth', Float32, self.raw_depth_callback)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Service('localization/zero_state', Trigger, lambda msg: self.handle_reset())

    def handle_reset(self):
        previous_offsets = f'{self.heave_offset=}, {self.yaw_offset=}, {self.pitch_offset=}, {self.roll_offset=}'

        self.heave_offset = None
        self.yaw_offset = None
        self.pitch_offset = None
        self.roll_offset = None

        self.heave_pub.publish(0)
        self.yaw_pub.publish(0)
        self.pitch_pub.publish(0)
        self.roll_pub.publish(0)

        return [True, previous_offsets]

    def raw_depth_callback(self, raw_depth: Float32):
        if self.heave_offset is None:
            self.heave_offset = raw_depth.data
        self.iekf_class.add_depth_measurement(raw_depth.data)
        self.publish_state() 

    def imu_callback(self, msg):

        orientation = msg.orientation

        quaternion = [
            orientation.x,
            orientation.y, 
            orientation.z, 
            orientation.w
        ]
        euler = euler_from_quaternion(quaternion)
        
        if self.yaw_offset is None:
            self.yaw_offset = degrees(-euler[2])
            self.pitch_offset = degrees(-euler[1])
            self.roll_offset = degrees(euler[0])

        yaw = degrees(-euler[2]) - self.yaw_offset
        pitch = degrees(-euler[1]) - self.pitch_offset
        roll = degrees(euler[0]) - self.roll_offset

        self.iekf_class.add_imu_measurement() #input should be measured acceleration and gyro (Imu.linear_acceleration ?)
        state = self.iekf_class.predict()
        self.publish_state()

    def dvl_callback(self, msg: Dvl):
        dvl_velocity = #array of x,y,z velocity components
        self.iekf_class.add_dvl_measurement(dvl_velocity)
        self.publish_state()


    def publish_state(self):
        state = self.iefk_class.predict()
        updated_yaw, updated_pitch, updated_roll = calculate_euler_angles_from_rotation_matrix(state.rotation)
        self.yaw_pub.publish(updated_yaw)
        self.pitch_pub.publish(updated_pitch) 
        self.roll_pub.publish(updated_roll) 
        self.heave_pub(state.position[2])
        self.x_pub.publish(state.position[0])
        self.y_pub.publish(state.position[1])

    def run(self):
        rospy.spin()

    def cleanup(self):
        pass

if __name__ == '__main__':
    StateEstimation().run()
