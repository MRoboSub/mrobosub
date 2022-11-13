#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pid import PIDController

from node import Node, Param

from typing import Optional, Final

class YawControlNode(Node):
    """
    Subscribers
    - /mrobosub/target_pose/yaw (deg)
    - /mrobosub/pose/yaw (deg)
    - /mrobosub/override_wrench/yaw (power)
    """

    """
    Publishers
    - /mrobosub/output_wrench/yaw
    """
    kP: Param[float]
    kI: Param[float]
    kD: Param[float]

    def __init__(self):
        super().__init__('heading_control')
        rospy.Subscriber('/mrobosub/target_pose/yaw', Float64, self.target_pose_callback)
        rospy.Subscriber('/mrobosub/pose/yaw', Float64, self.pose_callback)
        rospy.Subscriber('/mrobosub/override_wrench/yaw', Float64, self.override_wrench_callback)
        self.output_yaw_pub = rospy.Publisher('/mrobosub/output_wrench/yaw', Float64, queue_size=1)

        self.pid = PIDController(self.kP, self.kI, self.kD)
        self.target_pose: Optional[float] = None
        self.pose: Optional[float] = None
        
    def calculate(self):
        if self.target_pose is None or self.pose is None:
            return 0
        return self.pid.calculate_from_angle_setpoint(self.target_pose, self.pose)

    def target_pose_callback(self, target_pose: Float64):
        self.target_pose = target_pose.data
        self.output_yaw_pub.publish(self.calculate())

    def pose_callback(self, pose: Float64):
        self.pose = pose.data
        self.output_yaw_pub.publish(self.calculate())

    def override_wrench_callback(self, override_wrench: Float64):
        self.output_yaw_pub.publish(override_wrench.data)

    def run(self): 
        rospy.spin()

    def cleanup(self):
        self.output_yaw_pub.publish(0)

if __name__ == '__main__':
    YawControlNode().run()
