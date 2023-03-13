#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64
from typing import Callable, Dict, Union, Tuple, TypeVar
from enum import Enum, auto
from functools import partial

from mrobosub_lib.lib import Node, Param

config = Dict[str, Union[int, 'config']]
T = TypeVar('T')

class ControlMode(Enum):
    Twist = auto()
    Pose = auto()

class ButtonTrigger:
    def __init__(self, trigger: Callable[[], bool]) -> None:
        self.trigger = trigger
        self.last = self.trigger()

    def was_triggered(self) -> bool:
        """Should only be called once per period"""
        if self.trigger() is not self.last:
            self.last = not self.last
            return True
        return False

    def was_enabled(self) -> bool:
        """Should only be called once per period"""
        return self.was_triggered() and self.last

class Range:
    def __init__(self, lower: float, upper: float) -> None:
        self.lower = lower
        self.upper = upper

    def __call__(self, value: float) -> float:
        """Clamps value to between lower and upper"""
        return min(max(self.lower, value), self.upper)

class ModulusRange:
    def __init__(self, lower: float, upper: float) -> None:
        self.lower = lower
        self.upper = upper

    def __call__(self, value: float) -> float:
        """Ensures value is between lower and upper while preserving mod"""
        diff = self.upper - self.lower
        value = value % diff
        low_offset = self.lower % diff
        if value < low_offset:
            value += diff
        low_count = self.lower // diff
        return low_count * diff + value

def identity(x: T) -> T:
    return x


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

    surge: config
    sway: config
    heave: config
    roll: config
    pitch: config
    yaw: config
    estop_id: int

    def __init__(self) -> None:
        super().__init__("joystick_teleop")
        self.input_subscriber = rospy.Subscriber("/joy", Joy, self.joystick_callback)

        self.axes = []
        self.buttons = []
        self.pose = {'heave': 0}

        # Keys have no special meaning,
        # Only used to "name" the functions
        # so they can be accessed easily to modify
        self.periodic_funcs = {}

        self.config_pose_subscribers()
        self.enable_twist('surge')
        self.enable_twist('sway')

        self.init_toggleable_dof('heave', inital_setpoint=lambda: self.pose['heave'])
        self.init_toggleable_dof('yaw', inital_setpoint=lambda: self.pose['yaw'], pose_transform=ModulusRange(-180, 180))
        self.init_toggleable_dof('pitch', pose_transform=ModulusRange(-180, 180))
        self.init_toggleable_dof('roll', pose_transform=ModulusRange(-180, 180))

        self.should_stop = False
        def poll_estop():
            self.should_stop &= self.get_button(self.estop_id)
        self.periodic_funcs['poll_estop'] = poll_estop

    def joystick_callback(self, msg: Joy):
        self.axes = msg.axes
        self.buttons = msg.buttons
        print(msg)

    def config_pose_subscribers(self):
        self.pose_subscribers = []

        for dof in 'heave', 'yaw':
            def set_pose(pose: float):
                self.pose[dof] = pose
            self.pose_subscribers.append(rospy.Subscriber(f'/pose/{dof}', Float64, set_pose))

    def get_button(self, id: int) -> bool:
        if len(self.buttons) <= id:
            return False
        return self.buttons[id]

    def get_axis(self, id: int) -> float:
        if len(self.axes) <= id:
            return 0
        return self.axes[id]

    def periodic(self):
        for func in self.periodic_funcs.values():
            func()

    def create_twist_periodic(self, publish: Callable[[float], None], config: dict) -> Callable[[], None]:
        def periodic():
            output = 0
            if 'axis' in config:
                value = self.get_axis(config['axis']['id'])
                value *= config['axis'].get('scale', 1)
                output += value
            if 'increase' in config:
                if self.get_button(config['increase']['id']):
                    output += config['increase'].get('scale', 1)
            if 'decrease' in config:
                if self.get_button(config['decrease']['id']):
                    output -= config['decrease'].get('scale', 1)
            if 'invert' in config:
                if self.get_button(config['invert']):
                    output *= -1
            publish(output)

        periodic.mode = ControlMode.Twist
        return periodic

    def create_pose_periodic(self, publish: Callable[[float], None], config: dict, initial_setpoint=0) -> Callable[[], None]:
        setpoint = initial_setpoint

        if 'axis' in config:
            axis_id = config['axis']['id']
            scale = config['axis'].get('scale', 1)
            def get_axis():
                return self.get_axis(axis_id) * scale
        else:
            def get_axis():
                return 0

        if 'increase' in config:
            increase = ButtonTrigger(lambda: self.get_button(config['increase']['id']))
            scale = config['increase'].get('scale', 1)
            def get_increase():
                return increase.was_triggered() * scale
        else:
            def get_increase():
                return 0

        if 'decrease' in config:
            decrease = ButtonTrigger(lambda: self.get_button(config['decrease']['id']))
            scale = config['decrease'].get('scale', 1)
            def get_decrease():
                return decrease.was_triggered() * scale
        else:
            def get_decrease():
                return 0

        if 'invert' in config:
            invert_id = config['invert']['id']
            def get_invert():
                return self.get_button(invert_id)
        else:
            def get_invert():
                return False

        def periodic():
            nonlocal setpoint

            delta += get_axis()
            delta += get_increase()
            delta += get_decrease()
            if get_invert():
                delta *= -1

            setpoint += delta

            publish(setpoint)

        periodic.mode = ControlMode.Pose
        return periodic

    def enable_twist(self, dof: str):
        publisher = rospy.Publisher(f'/target_twist/{dof}', Float64, queue_size=1)
        self.periodic_funcs[dof] = self.create_twist_periodic(publisher.publish, getattr(self, dof)['twist'])

    def enable_pose(self, dof: str, initial_setpoint: Callable[[], float] = None, publish_transform: Callable[[float], float] = identity):
        publisher = rospy.Publisher(f'/target_pose/{dof}', Float64, queue_size=1)

        setpoint = initial_setpoint() if callable(initial_setpoint) else 0

        self.periodic_funcs[dof] = self.create_pose_periodic(
            lambda val: publisher.publish(publish_transform(val)),
            getattr(self, dof)['pose'],
            initial_setpoint=setpoint)

    def switch_mode(self, dof: str, inital_setpoint: Callable[[], float] = None, pose_transform: Callable[[float], float] = identity):
        print(f'Changing {dof} mode to', end=' ')
        if self.periodic_funcs[dof].mode == ControlMode.Twist:
            print('Twist')
            self.enable_twist(dof)
        else:
            print('Pose')
            self.enable_pose(dof, initial_setpoint=inital_setpoint, publish_transform=pose_transform)

    def init_toggleable_dof(self, dof: str, starting_mode: ControlMode = ControlMode.Twist, inital_setpoint: Callable[[], float] = None, pose_transform: Callable[[float], float] = identity):
        button_id = getattr(self, dof)['toggle']
        toggle_button = ButtonTrigger(lambda: self.get_button(button_id))

        def toggle_dof():
            if toggle_button.was_enabled():
                self.switch_mode(dof, inital_setpoint=inital_setpoint, pose_transform=pose_transform)

        self.periodic_funcs[f'toggle_{dof}'] = toggle_dof

        if starting_mode == ControlMode.Twist:
            self.enable_twist(dof)
        else:
            self.enable_pose(dof)

    def stop(self):
        for dof in 'surge', 'sway', 'heave', 'yaw', 'pitch', 'roll':
            rospy.Publisher(f'/target_twist/{dof}', Float64, queue_size=1).publish(0)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.should_stop:
                self.stop()
            else:
                self.periodic()
            rate.sleep()

    def cleanup(self):
        self.stop()

if __name__ == "__main__":
    JoystickTeleop().run()
