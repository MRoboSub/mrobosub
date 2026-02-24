"""
Button-driven joystick control scheme.

Buttons trigger discrete commands (increase/decrease/toggle); axes provide scale.
DOFs can toggle between POSE (position) and TWIST (velocity) mode.
"""

from enum import Enum
from typing import Tuple

from mrobosub_lib import Node
from std_msgs.msg import Float64


class ButtonCommand(int, Enum):
    """Discrete command triggered by a button."""

    INCREASE = 1
    DECREASE = -1
    TOGGLE = 0


class Button:
    """Tracks button state and parses command strings."""

    def __init__(self, idx: int, command_str: str) -> None:
        self.idx = idx
        self._is_pressed = False
        self._is_rising_edge = False
        self.name, self._action = self._split_command(command_str)

    @staticmethod
    def _split_command(comstr: str) -> Tuple[str, ButtonCommand]:
        """Split command string into (target_name, command)."""
        if comstr.endswith(".increase"):
            return (comstr[:-9], ButtonCommand.INCREASE)
        if comstr.endswith(".decrease"):
            return (comstr[:-9], ButtonCommand.DECREASE)
        if comstr.endswith(".toggle"):
            return (comstr[:-7], ButtonCommand.TOGGLE)
        if comstr == "estop" or comstr == "switch":
            return (comstr, ButtonCommand.TOGGLE)
        raise ValueError(f"Button command '{comstr}' not recognised")

    def update(self, is_pressed: bool) -> None:
        """Update state; rising edge = transition from not pressed to pressed."""
        self._is_rising_edge = (not self._is_pressed) and is_pressed
        self._is_pressed = is_pressed

    @property
    def rising_edge(self) -> bool:
        return self._is_rising_edge

    @property
    def pressed(self) -> bool:
        return self._is_pressed

    @property
    def command(self) -> Tuple[str, ButtonCommand]:
        return self.name, self._action


class JoystickDOF:
    """
    DOF with toggleable POSE/TWIST mode.

    Subscribes to /pose/{name} for feedback. Publishes to target_twist/pose.
    """

    class DOFState(int, Enum):
        POSE = 0
        TWIST = 1

    def __init__(
        self,
        axis_idx: int,
        name: str,
        scale_twist: float,
        scale_pose: float,
        is_toggleable: bool,
        node: Node,
    ) -> None:
        self.axis_idx = axis_idx
        self.name = name
        self.scale_twist = scale_twist
        self.scale_pose = scale_pose
        self.is_toggleable = is_toggleable

        self.pos = 0.0
        self.state = self.DOFState.TWIST
        self.scale = 0.0
        self.setpoint = 0.0

        self._twist_pub = node.create_publisher(
            Float64, f"/target_twist/{name}", qos_profile=1
        )
        self._pose_pub = node.create_publisher(
            Float64, f"/target_pose/{name}", qos_profile=1
        )
        node.create_subscription(
            Float64, f"/pose/{name}", self._pose_callback, qos_profile=1
        )

    def _pose_callback(self, msg: Float64) -> None:
        self.pos = msg.data

    def update(self, command: ButtonCommand, axis_value: float) -> None:
        """Update state from button command; axis_value is the axis (scale)."""
        if command == ButtonCommand.TOGGLE and self.is_toggleable:
            self.state = (
                self.DOFState.POSE
                if self.state == self.DOFState.TWIST
                else self.DOFState.TWIST
            )

        mag = abs(axis_value)
        if self.state == self.DOFState.TWIST:
            if command == ButtonCommand.INCREASE:
                self.scale = self.scale_twist * mag
            elif command == ButtonCommand.DECREASE:
                self.scale = -self.scale_twist * mag
        else:
            delta = self.scale_pose * mag
            if command == ButtonCommand.INCREASE:
                self.setpoint = self.pos + delta
            elif command == ButtonCommand.DECREASE:
                self.setpoint = self.pos - delta
            else:
                self.setpoint = self.pos + (delta if axis_value >= 0 else -delta)

    def publish(self) -> None:
        """Publish current command to target_twist or target_pose."""
        if self.state == self.DOFState.POSE:
            self._pose_pub.publish(Float64(data=self.setpoint))
        else:
            self._twist_pub.publish(Float64(data=self.scale))

    def publish_zero_twist(self) -> None:
        """Publish zero to target_twist (reset/estop)."""
        self._twist_pub.publish(Float64(data=0.0))
