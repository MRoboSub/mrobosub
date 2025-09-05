#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

from typing import Optional, Final

"""
expose subscribers
    /control_effort
expose publishers
    /setpoint
    /state
    /pid_enable
"""

class PIDInterface():
    def __init__(self, node: Node, pid_name: str, callback):
        self.subscriber = node.create_subscription(Float64, f'/{pid_name}/control_effort', self.control_effort_callback, qos_profile=1)
        self.publisher_enable = node.create_publisher(Bool, f'/{pid_name}/pid_enable', qos_profile=1)
        self.publisher_setpoint = node.create_publisher(Float64, f'/{pid_name}/setpoint', qos_profile=1)
        self.publisher_state = node.create_publisher(Float64, f'/{pid_name}/state', qos_profile=1)
        self.effort = 0
        self.callback = callback

    def control_effort_callback(self, effort: Float64):
        self.effort = effort.data
        self.callback(effort.data)

    def get_effort(self):
        return self.effort

    def set_current(self, current):
        self.publisher_state.publish(current)

    def set_target(self, target):
        self.publisher_enable.publish(True)
        self.publisher_setpoint.publish(target)

    def disable(self):
        self.publisher_enable.publish(False)