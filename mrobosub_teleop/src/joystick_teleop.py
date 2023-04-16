#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64
from std_srvs.srv import Trigger
from typing import Callable, Dict, Union
from enum import Enum, auto

from mrobosub_lib.lib import Node, Param

config = Dict[str, Union[int, bool, 'config']]

AXES = 'surge', 'sway', 'heave', 'yaw', 'roll', 'pitch'

class ControlMode(Enum):
    Twist = auto()
    Pose = auto()

class ButtonTrigger:
    def __init__(self, trigger: Callable[[], bool]) -> None:
        self.trigger = trigger
        self.last = self.trigger()

    def dual_edge(self) -> bool:
        """Should only be called once per period"""
        if self.trigger() != self.last:
            self.last = not self.last
            return True
        return False

    def rising_edge(self) -> bool:
        """Should only be called once per period"""
        return self.dual_edge() and self.last

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

class Inputs:
    def __init__(self):
        self.axes = []
        self.buttons = []

    def get_axis(self, id: int) -> float:
        if len(self.axes) <= id:
            return 0
        return self.axes[id]

    def get_button(self, id: int) -> bool:
        if len(self.buttons) <= id:
            return False

        return bool(self.buttons[id])
class DOF:
    def __init__(self, inputs: Inputs, config: config, twist_pub: str) -> None:
        self.inputs = inputs
        self.config = config
        self.twist_pub = rospy.Publisher(twist_pub, Float64, queue_size=1)

    def get_twist_input(self) -> float:
        config = self.config['twist']
        output = 0
        if 'axis' in config:
            value = self.inputs.get_axis(config['axis']['id'])
            value *= config['axis'].get('scale', 1)
            output += value
        if 'increase' in config:
            if self.inputs.get_button(config['increase']['id']):
                output += config['increase'].get('scale', 1)
        if 'decrease' in config:
            if self.inputs.get_button(config['decrease']['id']):
                output -= config['decrease'].get('scale', 1)
        if 'invert' in config:
            if self.inputs.get_button(config['invert']):
                output *= -1
        return output

    def pose_delta_factory(self) -> Callable[[], float]:
        config = self.config['pose']

        if 'axis' in config:
            axis_id = config['axis']['id']
            axis_scale = config['axis'].get('scale', 1)
            def get_axis():
                return self.inputs.get_axis(axis_id) * axis_scale
        else:
            def get_axis():
                return 0

        if 'increase' in config:
            increase = ButtonTrigger(lambda: self.inputs.get_button(config['increase']['id']))
            inc_scale = config['increase'].get('scale', 1)
            def get_increase():
                return increase.rising_edge() * inc_scale
        else:
            def get_increase():
                return 0

        if 'decrease' in config:
            decrease = ButtonTrigger(lambda: self.inputs.get_button(config['decrease']['id']))
            dec_scale = config['decrease'].get('scale', 1) * -1
            def get_decrease():
                return decrease.rising_edge() * dec_scale
        else:
            def get_decrease():
                return 0

        if 'invert' in config:
            invert_id = config['invert']['id']
            def get_invert():
                return self.inputs.get_button(invert_id)
        else:
            def get_invert():
                return False

        def get_delta() -> float:
            delta = 0
            delta += get_axis()
            delta += get_increase()
            delta += get_decrease()
            if get_invert():
                delta *= -1
            return delta

        return get_delta

    def __call__(self) -> None:
        ...


class ToggleableDOF(DOF):
    def __init__(self, inputs: Inputs, config: config, twist_pub: str, pose_pub: str) -> None:
        super().__init__(inputs, config, twist_pub)
        self.toggle_button = ButtonTrigger(lambda: inputs.get_button(config['toggle']))
        self.pose_pub = rospy.Publisher(pose_pub, Float64, queue_size=1)


class SurgeControl(DOF):
    def __init__(self, inputs: Inputs, config: config) -> None:
        super().__init__(inputs, config, '/target_twist/surge')

    def __call__(self):
        self.twist_pub.publish(self.get_twist_input())


class SwayControl(DOF):
    def __init__(self, inputs: Inputs, config: config) -> None:
        super().__init__(inputs, config, '/target_twist/sway')

    def __call__(self):
        self.twist_pub.publish(self.get_twist_input())


class HeaveControl(ToggleableDOF):
    def __init__(self, inputs: Inputs, config: config) -> None:
        super().__init__(inputs, config, '/target_twist/heave', '/target_pose/heave')
        self.mode = ControlMode.Twist
        self.pose = 0
        self.pose_sub = rospy.Subscriber('/pose/heave', Float64, self.update_pose)
        self.get_pose_delta = self.pose_delta_factory()

    def update_pose(self, new_pose: Float64):
        self.pose = new_pose.data

    def __call__(self) -> None:
        if self.toggle_button.rising_edge():
            if self.mode == ControlMode.Twist:
                self.mode = ControlMode.Pose
                self.setpoint = self.pose
            else:
                self.mode = ControlMode.Twist

        if self.mode == ControlMode.Twist:
            self.twist_pub.publish(self.get_twist_input())
        else:
            self.setpoint += self.get_pose_delta()
            self.pose_pub.publish(self.setpoint)


class YawControl(ToggleableDOF):
    def __init__(self, inputs: Inputs, config: config) -> None:
        super().__init__(inputs, config, '/target_twist/yaw', '/target_pose/yaw')
        self.mode = ControlMode.Twist
        self.pose = 0
        self.pose_sub = rospy.Subscriber('/pose/yaw', Float64, self.update_pose)
        self.get_pose_delta = self.pose_delta_factory()
        self.range = ModulusRange(-180, 180)

    def update_pose(self, new_pose: Float64):
        self.pose = new_pose.data

    def __call__(self) -> None:
        if self.toggle_button.rising_edge():
            if self.mode == ControlMode.Twist:
                self.setpoint = self.pose
                self.mode = ControlMode.Pose
            else:
                self.mode = ControlMode.Twist

        if self.mode == ControlMode.Twist:
            self.twist_pub.publish(self.get_twist_input())
        else:
            self.setpoint += self.get_pose_delta()
            self.setpoint = self.range(self.setpoint)
            self.pose_pub.publish(self.setpoint)


class RollControl(ToggleableDOF):
    def __init__(self, inputs: Inputs, config: config) -> None:
        super().__init__(inputs, config, '/target_twist/roll', '/target_pose/roll')
        self.mode = ControlMode.Twist
        self.get_pose_delta = self.pose_delta_factory()
        self.range = ModulusRange(-180, 180)

    def __call__(self) -> None:
        if self.toggle_button.rising_edge():
            if self.mode == ControlMode.Twist:
                self.setpoint = 0
                self.mode = ControlMode.Pose
            else:
                self.mode = ControlMode.Twist

        if self.mode == ControlMode.Twist:
            self.twist_pub.publish(self.get_twist_input())
        else:
            self.setpoint += self.get_pose_delta()
            self.setpoint = self.range(self.setpoint)
            self.pose_pub.publish(self.setpoint)


class PitchControl(ToggleableDOF):
    def __init__(self, inputs: Inputs, config: config) -> None:
        super().__init__(inputs, config, '/target_twist/pitch', '/target_pose/pitch')
        self.mode = ControlMode.Twist
        self.get_pose_delta = self.pose_delta_factory()
        self.range = ModulusRange(-180, 180)

    def __call__(self) -> None:
        if self.toggle_button.rising_edge():
            if self.mode == ControlMode.Twist:
                self.setpoint = 0
                self.mode = ControlMode.Pose
            else:
                self.mode = ControlMode.Twist

        if self.mode == ControlMode.Twist:
            self.twist_pub.publish(self.get_twist_input())
        else:
            self.setpoint += self.get_pose_delta()
            self.setpoint = self.range(self.setpoint)
            self.pose_pub.publish(self.setpoint)


class StateMachineMode:
    def __init__(self, inputs: Inputs, config: config, handle_soft_stop: Callable[[], None]) -> None:
        self.switch_mode_button = ButtonTrigger(lambda: inputs.get_button(config['switch_mode']))
        self.soft_stop_button = ButtonTrigger(lambda: inputs.get_button(config['soft_stop_id']))
        self.zero_button = ButtonTrigger(lambda: inputs.get_button(config['zero_pos_sensors']))
        self.handle_soft_stop = handle_soft_stop

    def __call__(self):
        if self.soft_stop_button.rising_edge():
            self.handle_soft_stop()

        if self.zero_button.rising_edge():
            zero = rospy.ServiceProxy('localization/zero_state', Trigger)
            try:
                res = zero()
                print('Zeroed state estimator')
                print(res)
            except rospy.ServiceException as exc:
                print(f'Unable to zero state estimator ({exc})')

    def should_switch_mode(self) -> bool:
        return self.switch_mode_button.rising_edge()

class JoystickTeleop(Node):
    """
    Publishers
    - /target_pose/heave
    - /target_twist/heave
    - /output_wrench/heave
    - /target_twist/surge
    - /output_wrench/surge
    - /target_twist/sway
    - /output_wrench/sway
    - /target_pose/yaw
    - /target_twist/yaw
    - /output_wrench/yaw
    - /target_pose/roll
    - /target_twist/roll
    - /output_wrench/roll
    - /target_pose/pitch
    - /target_twist/pitch
    - /output_wrench/pitch

    Subscribers
    - /joy
    - /pose/heave
    - /pose/yaw
    """

    surge: config
    sway: config
    heave: config
    roll: config
    pitch: config
    yaw: config
    state_machine: config
    estop_id: int
    update_rate: int

    def __init__(self) -> None:
        super().__init__("joystick_teleop")
        self.input_subscriber = rospy.Subscriber("/joy", Joy, self.joystick_callback)

        self.inputs = Inputs()
        self.pose = {'heave': 0}

        # Keys have no special meaning,
        # Only used to "name" the functions
        # so they can be accessed easily to modify
        self.periodic_funcs = {}
        self.axis_controls: dict[str, DOF] = {}

        self.axis_controls['surge'] = SurgeControl(self.inputs, self.surge)
        self.axis_controls['sway'] = SwayControl(self.inputs, self.sway)
        self.axis_controls['heave'] = HeaveControl(self.inputs, self.heave)
        self.axis_controls['yaw'] = YawControl(self.inputs, self.yaw)
        self.axis_controls['roll'] = RollControl(self.inputs, self.roll)
        self.axis_controls['pitch'] = PitchControl(self.inputs, self.pitch)
        self.periodic_funcs.update(self.axis_controls)

        self.wrench_pubs = [rospy.Publisher(f'/output_wrench/{axis}', Float64, queue_size=1) for axis in AXES]

        self.state_machine_mode = StateMachineMode(self.inputs, self.state_machine, self.soft_stop)

        self.estop_trigger = ButtonTrigger(lambda: self.inputs.get_button(self.estop_id))

        self.stop = False
        def poll_hard_estop():
            if self.estop_trigger.rising_edge():
                self.soft_stop()
                self.stop = True
        self.periodic_funcs['poll_estop'] = poll_hard_estop

        self.in_state_machine_mode = False
        def poll_switch_sm_mode():
            if self.state_machine_mode.should_switch_mode():
                self.switch_sm_mode()
        self.periodic_funcs['poll_switch_sm_mode'] = poll_switch_sm_mode

        print('Teleop startup complete')

    def joystick_callback(self, msg: Joy):
        self.inputs.axes = msg.axes
        self.inputs.buttons = msg.buttons

    def periodic(self):
        for func in list(self.periodic_funcs.values()):
            func()

    def hard_stop(self):
        for axis in self.axis_controls.values():
            axis.twist_pub.publish(0)
        for pub in self.wrench_pubs:
            pub.publish(0)

    def switch_sm_mode(self):
        if not self.in_state_machine_mode:
            for axis in AXES:
                del self.periodic_funcs[axis]
            self.periodic_funcs['state_machine'] = self.state_machine_mode
            self.in_state_machine_mode = True
            print('Quitting teleop mode, entering state machine mode')
        else:
            del self.periodic_funcs['state_machine']
            self.periodic_funcs.update(self.axis_controls)
            self.in_state_machine_mode = False
            print('Quitting state machine mode, entering teleop mode')

    def soft_stop(self):
        soft_stop = rospy.ServiceProxy('captain/soft_stop', Trigger)
        try:
            res = soft_stop()
            print('Soft stopped state machine:')
            print(res)
        except rospy.ServiceException as exc:
            print(f'Unable to process soft stop ({exc})')

    def run(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            if self.stop:
                self.hard_stop()
            else:
                self.periodic()
            rate.sleep()

    def cleanup(self):
        self.hard_stop()

if __name__ == "__main__":
    JoystickTeleop().run()
