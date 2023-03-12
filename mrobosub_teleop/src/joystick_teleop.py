#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64
from typing import Callable, Dict, Union
from enum import Enum, auto

from mrobosub_lib.lib import Node, Param

config = Dict[str, Union[int, 'config']]

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

def do_nothing():
    pass

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
        self.input_subscriber = rospy.Subscriber("/joy", Joy, self.callback)

        self.axes = []
        self.buttons = []

        self.publisher_map = {}

        self.periodic_funcs = {}

        self.enable_twist('surge')
        self.enable_twist('sway')
        self.init_toggleable_dof('heave')
        self.init_toggleable_dof('yaw')
        self.init_toggleable_dof('pitch')
        self.init_toggleable_dof('roll')

        self.should_stop = False
        def poll_estop():
            self.should_stop &= self.get_button(self.estop_id)
        self.periodic_funcs['poll_estop'] = poll_estop

        # surge_publisher = self.get_publisher('/target_twist/surge')
        # self.periodic_funcs['surge'] = self.create_twist_periodic(surge_publisher.publish, self.surge)
        # sway_publisher = self.get_publisher('/target_twist/sway')
        # self.periodic_funcs['sway'] = self.create_twist_periodic(sway_publisher.publish, self.sway)
        # self.periodic_funcs['heave'] = do_nothing
        # self.periodic_funcs['roll'] = do_nothing
        # self.periodic_funcs['pitch'] = do_nothing
        # self.periodic_funcs['yaw'] = do_nothing

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

    def get_publisher(self, topic: str) -> rospy.Publisher:
        if topic not in self.publisher_map:
            self.publisher_map[topic] = rospy.Publisher(topic, Float64, queue_size=1)
        return self.publisher_map[topic]

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
            publish(output)

        periodic.mode = ControlMode.Twist
        return periodic

    def create_pose_periodic(self, publish: Callable[[float], None], config: dict, initial_setpoint=0) -> Callable[[], None]:
        setpoint = initial_setpoint

        def get_axis():
            return 0
        if 'axis' in config:
            axis_id = config['axis']['id']
            scale = config['axis'].get('scale', 1)
            def get_axis():
                return self.get_axis(axis_id) * scale

        def get_increase():
            return 0
        if 'increase' in config:
            increase = ButtonTrigger(lambda: self.get_button(config['increase']['id']))
            def get_increase():
                return increase.was_triggered() * config['increase'].get('scale', 1)

        def get_decrease():
            return 0
        if 'decrease' in config:
            decrease = ButtonTrigger(lambda: self.get_button(config['decrease']['id']))
            def get_decrease():
                return decrease.was_triggered() * config['decrease'].get('scale', 1)

        def get_invert():
            return False
        if 'invert' in config:
            invert_id = config['invert']['id']
            def get_invert():
                return self.get_button(invert_id)

        def periodic():
            nonlocal setpoint

            setpoint += get_axis()
            setpoint += get_increase()
            setpoint += get_decrease()
            if get_invert():
                setpoint *= -1

            publish(setpoint)

        periodic.mode = ControlMode.Pose
        return periodic

    def enable_twist(self, dof: str):
        publisher = self.get_publisher(f'/target_twist/{dof}')
        self.periodic_funcs[dof] = self.create_twist_periodic(publisher.publish, getattr(self, dof)['twist'])

    def enable_pose(self, dof: str):
        publisher = self.get_publisher(f'/target_pose/{dof}')
        self.periodic_funcs[dof] = self.create_pose_periodic(publisher.publish, getattr(self, dof)['pose'])

    def switch_mode(self, dof: str):
        print(f'Changing {dof} mode to', end=' ')
        if self.periodic_funcs[dof].mode == ControlMode.Twist:
            print('Twist')
            self.enable_pose(dof)
        else:
            print('Pose')
            self.enable_twist(dof)

    def init_toggleable_dof(self, dof: str, starting_mode: ControlMode = ControlMode.Twist):
        button_id = getattr(self, dof)['toggle']
        toggle_button = ButtonTrigger(lambda: self.get_button(button_id))

        def toggle_dof():
            if toggle_button.was_enabled():
                self.switch_mode(dof)

        self.periodic_funcs[f'toggle_{dof}'] = toggle_dof

        if starting_mode == ControlMode.Twist:
            self.enable_twist(dof)
        else:
            self.enable_pose(dof)

    def stop(self):
        for dof in 'surge', 'sway', 'heave', 'yaw', 'pitch', 'roll':
            self.get_publisher(f'/target_twist/{dof}').publish(0)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.should_stop:
                self.stop()
            else:
                self.periodic()
            rate.sleep()

if __name__ == "__main__":
    JoystickTeleop().run()
