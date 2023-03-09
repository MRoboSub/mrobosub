#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64
from genpy import Message
from math import atan2

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

    # JSON-like objects
    surge: ...
    sway: ...
    heave: ...
    roll: ...
    pitch: ...
    yaw: ...

    def __init__(self):
        super().__init__("joystick_teleop")
        self.joystick_subscriber = rospy.Subscriber("/joy", Joy, self.callback)

        self.periodic_funcs = {}
        self.periodic_funcs['surge'] = lambda: None
        self.periodic_funcs['sway'] = lambda: None
        self.periodic_funcs['heave'] = lambda: None
        self.periodic_funcs['roll'] = lambda: None
        self.periodic_funcs['pitch'] = lambda: None
        self.periodic_funcs['yaw'] = lambda: None

        self.axes = []
        self.buttons = []

        self.publisher_map = {}

    def callback(self, msg: Joy):
        self.axes = msg.axes
        self.buttons = msg.buttons
        print(msg)

    def get_button(self, id: int) -> bool:
        if len(self.buttons) <= id:
            return False
        return self.buttons[id]

    def get_axis(self, id: int) -> float:
        if len(self.axes) <= id:
            return 0
        return self.axes[id]

    def get_publisher(self, topic: str, msg_type: Message) -> rospy.Publisher:
        if topic not in self.publisher_map:
            self.publisher_map[topic] = rospy.Publisher(topic, msg_type, queue_size=1)
        return self.publisher_map[topic]

    def periodic(self):
        for func in self.periodic_funcs.values():
            func()

    def enable_joystick_yaw(self):
        yaw_publisher = self.get_publisher("/target_pose/yaw", Float64)
        def periodic():
            angle = atan2() # TODO
            yaw_publisher.publish(angle)
        self.periodic_funcs['yaw'] = periodic

    def print_controls(self):
        print(self.surge)
        print(self.sway)
        print(self.heave)
        print(self.yaw)

    def run(self):
        while not rospy.is_shutdown():
            self.periodic()
            rospy.sleep(0.05)

if __name__ == "__main__":
    JoystickTeleop().run()
