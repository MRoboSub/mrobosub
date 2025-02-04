#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float32

from mrobosub_lib.lib import Node, Param

from typing import Tuple
from typing_extensions import Annotated, Literal, TypeAlias

from std_srvs.srv import Trigger
from mrobosub_msgs.msg import Imu, Dvl, Iekf as IekfMsg

from tf.transformations import euler_from_matrix
from math import degrees

import constants
from iekf import IEKF, State

import numpy as np
import numpy.typing as npt

Mat3x3: TypeAlias = Annotated[npt.NDArray[np.float64], Literal[3, 3]]

def decompose_matrix(matrix: Mat3x3) -> Tuple[float, float, float]:
    yaw, pitch, roll = euler_from_matrix(matrix, 'rzyx')
    return degrees(yaw), degrees(pitch), degrees(roll)

class StateEstimation(Node):
    """
    Subscribers
    - /imu/data
    - /dvl/translational_data
    - /depth/raw_depth
    """

    """
    Publishers
    - /pose/x
    - /pose/y
    - /pose/heave
    - /pose/yaw
    - /pose/pitch
    - /pose/roll
    - /pose
    """

    def __init__(self):
        super().__init__('localization')
        init_cov = np.diag([
            np.pi/6, np.pi/6, np.pi/6,
            1, 1, 1,
            .1, .1, .1,
            .005, .005, .005,
            .05, .05, .05])
        self.iekf = IEKF(constants.DefaultConstants(), State.identity(), init_cov)
        self.zero_state = State.identity()

        self.x_pub = rospy.Publisher('/pose/x_pos', Float64, queue_size=1)
        self.y_pub = rospy.Publisher('/pose/y_pos', Float64, queue_size=1)
        self.heave_pub = rospy.Publisher('/pose/heave', Float64, queue_size=1)
        self.yaw_pub = rospy.Publisher('/pose/yaw', Float64, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pose/pitch', Float64, queue_size=1)
        self.roll_pub = rospy.Publisher('/pose/roll', Float64, queue_size=1) 
        self.pose_pub = rospy.Publisher('/pose', Iekf, queue_size=1)

        rospy.Subscriber('/dvl/translational_data', Dvl, self.dvl_callback)   
        rospy.Subscriber('/depth/raw_depth', Float32, self.depth_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        rospy.Service('localization/zero_state', Trigger, lambda _msg: self.handle_reset())

    def handle_reset(self):
        previous_state = f'{self.iekf.predict()=}'
        self.zero_state = self.iekf.predict()
        self.publish_state()
        return [True, previous_state]

    def imu_callback(self, msg: Imu):
        acc = np.array([msg.linAccA, msg.linAccB, msg.linAccC])
        gyro = np.array([msg.angVelA, msg.angVelB, msg.angVelC])

        self.iekf.add_imu_measurement(acc, gyro, msg.dt)
        self.publish_state()

    def dvl_callback(self, msg: Dvl):
        vel = np.array([msg.velocityA, msg.velocityB, msg.velocityC])
        self.iekf.add_dvl_measurement(vel)
        self.publish_state()

    def depth_callback(self, raw_depth: Float32):
        self.iekf.add_depth_measurement(raw_depth.data)
        self.publish_state() 

    def publish_state(self):
        state = self.iekf.predict()
        yaw, pitch, roll = decompose_matrix(self.zero_state.rotation.T @ state.rotation)
        relative_state = IekfMsg(
            yaw = yaw % 360,
            pitch = pitch % 360,
            roll = roll % 360,
            twist_x = state.velocity[0],
            twist_y = state.velocity[1],
            twist_heave = state.velocity[2],
            pose_x = state.position[0] - self.zero_state.position[0],
            pose_y = state.position[1] - self.zero_state.position[1],
            pose_heave = state.position[2] - self.zero_state.position[2],
        )
        self.yaw_pub.publish(relative_state.yaw)
        self.pitch_pub.publish(relative_state.pitch)
        self.roll_pub.publish(relative_state.roll)
        self.x_pub.publish(relative_state.pose_x)
        self.y_pub.publish(relative_state.pose_y)
        self.heave_pub.publish(relative_state.pose_heave)
        self.pose_pub.publish(relative_state)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    StateEstimation().run()
