#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

"""
expose subscribers
    /control_effort
expose publishers
    /setpoint
    /state
"""

class PIDInterface():
    def __init__(self, pid_name, callback):
        rospy.Subscriber(f'/{pid_name}/control_effort', Float64, self.control_effort_callback)
        self.publisher_setpoint = rospy.Publisher(f'/{pid_name}/setpoint', Float64, queue_size=1)
        self.publisher_state = rospy.Publisher(f'/{pid_name}/state', Float64, queue_size=1)
        self.effort = 0
        self.callback = callback;

    def control_effort_callback(self, effort: Float64):
        self.effort = effort.data
        self.callback(effort.data)

    def get_effort(self):
        return self.effort

    def set_current(self, current):
        self.publisher_state.publish(current)

    def set_target(self, target):
        self.publisher_setpoint.publish(target)