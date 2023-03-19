#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

from mrobosub_lib.lib import Node, Param


class JoystickTeleop(Node):
    """
    Publishers
    - /target_pose/heave
    - /target_twist/heave
    - /target_twist/surge
    - /target_twist/sway
    - /target_pose/yaw
    - /target_twist/yaw
    - /target_pose/roll
    - /target_twist/roll
    - /target_pose/pitch
    - /target_twist/pitch

    Subscribers
    - /joy
    """

    surge: ...
    sway: ...
    heave: ...
    yaw: ...

    def __init__(self):
        super().__init__("joystick_teleop")
        self.joystick_subscriber = rospy.Subscriber("/joy", Joy, self.callback)
        self.depth_to_hold = None
        self.yaw_to_hold = None
        self.roll_to_hold = None
        self.pitch_to_hold = None

    def callback(self, msg: Joy):
        print(msg.axes, msg.buttons)
        print(f'{self.surge}')
        print(f'{self.sway}')
        print(f'{self.heave}')
        print(f'{self.yaw}')

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    JoystickTeleop().run()
