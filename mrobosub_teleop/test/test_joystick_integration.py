# Copyright 2025 Michigan Robotic Submarine
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Integration test for joystick teleop (button-driven and continuous modes).

Runs joystick_teleop / joystick_teleop_continuous with mock Joy messages.
"""

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class JoyPublisher(Node):
    """Publishes mock Joy messages for testing."""

    def __init__(self):
        super().__init__("joy_publisher")
        self.pub = self.create_publisher(Joy, "/joy", 10)

    def publish_joy(self, axes: list[float], buttons: list[int] | None = None):
        msg = Joy()
        msg.axes = axes
        msg.buttons = buttons if buttons is not None else [0] * 8
        self.pub.publish(msg)


class TwistCollector(Node):
    """Subscribes to target_twist and stores last received values."""

    def __init__(self):
        super().__init__("twist_collector")
        self.values = {}
        for dof in ["surge", "sway", "heave", "yaw", "roll", "pitch"]:
            self.create_subscription(
                Float64,
                f"/target_twist/{dof}",
                lambda msg, d=dof: self._callback(msg, d),
                10,
            )
            self.values[dof] = None

    def _callback(self, msg: Float64, dof: str):
        self.values[dof] = msg.data


def test_joystick_teleop_button_increase():
    """Verify surge.increase button with axis scale produces target_twist/surge."""
    from mrobosub_teleop.joystick_teleop import JoystickTeleop

    if not rclpy.ok():
        rclpy.init()
    teleop = publisher = collector = None
    try:
        teleop = JoystickTeleop()
        publisher = JoyPublisher()
        collector = TwistCollector()

        executor = SingleThreadedExecutor()
        executor.add_node(teleop)
        executor.add_node(publisher)
        executor.add_node(collector)

        # First: no buttons pressed (establish baseline)
        axes = [0.0, 0.6, 0.0, 0.0, 0.0, 0.0]
        for _ in range(5):
            publisher.publish_joy(axes, buttons=[0] * 10)
            executor.spin_once(timeout_sec=0.02)

        # Then: button 0 (surge.increase) rising edge, axis 1 (surge) = 0.6
        # Expected: scale_t 0.5 * 0.6 = 0.3
        publisher.publish_joy(axes, buttons=[1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        for _ in range(30):
            executor.spin_once(timeout_sec=0.02)

        assert collector.values["surge"] is not None
        assert abs(collector.values["surge"] - 0.3) < 0.05

    finally:
        for node in (teleop, publisher, collector):
            if node is not None:
                node.destroy_node()


def test_joystick_teleop_estop_zeros_output():
    """Verify estop button causes all target_twist to go to zero."""
    from mrobosub_teleop.joystick_teleop import JoystickTeleop

    if not rclpy.ok():
        rclpy.init()
    teleop = publisher = collector = None
    try:
        teleop = JoystickTeleop()
        publisher = JoyPublisher()
        collector = TwistCollector()

        executor = SingleThreadedExecutor()
        executor.add_node(teleop)
        executor.add_node(publisher)
        executor.add_node(collector)

        # First: trigger surge.increase
        axes = [0.0, 0.6, 0.0, 0.0, 0.0, 0.0]
        for _ in range(5):
            publisher.publish_joy(axes, buttons=[0] * 10)
            executor.spin_once(timeout_sec=0.02)
        publisher.publish_joy(axes, buttons=[1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        for _ in range(10):
            executor.spin_once(timeout_sec=0.02)

        # Then: press estop (button 4)
        estop_buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
        max_spins = 150
        for _ in range(max_spins):
            publisher.publish_joy(axes, buttons=estop_buttons)
            executor.spin_once(timeout_sec=0.02)
            if all(
                abs(collector.values.get(d, 1) or 0) < 0.01
                for d in ["surge", "sway", "heave", "yaw"]
            ):
                break
        else:
            pytest.fail(
                f"Estop did not zero outputs within {max_spins} spins. "
                f"surge={collector.values.get('surge')}"
            )

    finally:
        for node in (teleop, publisher, collector):
            if node is not None:
                node.destroy_node()


def test_joystick_teleop_rising_edge_only():
    """Verify zero axis produces zero output (button with no scale)."""
    from mrobosub_teleop.joystick_teleop import JoystickTeleop

    if not rclpy.ok():
        rclpy.init()
    teleop = publisher = collector = None
    try:
        teleop = JoystickTeleop()
        publisher = JoyPublisher()
        collector = TwistCollector()

        executor = SingleThreadedExecutor()
        executor.add_node(teleop)
        executor.add_node(publisher)
        executor.add_node(collector)

        # surge.increase with axis 1 = 0 (zero scale)
        axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for _ in range(5):
            publisher.publish_joy(axes, buttons=[0] * 10)
            executor.spin_once(timeout_sec=0.02)
        publisher.publish_joy(axes, buttons=[1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        for _ in range(20):
            executor.spin_once(timeout_sec=0.02)

        assert collector.values["surge"] is not None
        assert abs(collector.values["surge"]) < 0.01

    finally:
        for node in (teleop, publisher, collector):
            if node is not None:
                node.destroy_node()


# --- Continuous mode tests ---


def test_joystick_teleop_continuous_surge_output():
    """Verify surge axis maps directly to target_twist/surge (continuous mode)."""
    from mrobosub_teleop.joystick_teleop_continuous import JoystickTeleopContinuous

    if not rclpy.ok():
        rclpy.init()
    teleop = publisher = collector = None
    try:
        teleop = JoystickTeleopContinuous()
        publisher = JoyPublisher()
        collector = TwistCollector()

        executor = SingleThreadedExecutor()
        executor.add_node(teleop)
        executor.add_node(publisher)
        executor.add_node(collector)

        # Axis 1 (surge) = 0.6, scale 0.5 -> 0.3
        axes = [0.0, 0.6, 0.0, 0.0, 0.0, 0.0]
        for _ in range(30):
            publisher.publish_joy(axes)
            executor.spin_once(timeout_sec=0.02)

        assert collector.values["surge"] is not None
        assert abs(collector.values["surge"] - 0.3) < 0.05

    finally:
        for node in (teleop, publisher, collector):
            if node is not None:
                node.destroy_node()


def test_joystick_teleop_continuous_deadzone():
    """Verify small joystick input below deadzone produces zero."""
    from mrobosub_teleop.joystick_teleop_continuous import JoystickTeleopContinuous

    if not rclpy.ok():
        rclpy.init()
    teleop = publisher = collector = None
    try:
        teleop = JoystickTeleopContinuous()
        publisher = JoyPublisher()
        collector = TwistCollector()

        executor = SingleThreadedExecutor()
        executor.add_node(teleop)
        executor.add_node(publisher)
        executor.add_node(collector)

        # Axis 1 = 0.05 (below deadzone 0.1)
        axes = [0.0, 0.05, 0.0, 0.0, 0.0, 0.0]
        for _ in range(30):
            publisher.publish_joy(axes)
            executor.spin_once(timeout_sec=0.02)

        assert collector.values["surge"] is not None
        assert abs(collector.values["surge"]) < 0.01

    finally:
        for node in (teleop, publisher, collector):
            if node is not None:
                node.destroy_node()


def test_joystick_teleop_continuous_estop():
    """Verify estop button zeros all outputs in continuous mode."""
    from mrobosub_teleop.joystick_teleop_continuous import JoystickTeleopContinuous

    if not rclpy.ok():
        rclpy.init()
    teleop = publisher = collector = None
    try:
        teleop = JoystickTeleopContinuous()
        publisher = JoyPublisher()
        collector = TwistCollector()

        executor = SingleThreadedExecutor()
        executor.add_node(teleop)
        executor.add_node(publisher)
        executor.add_node(collector)

        # First: non-zero surge
        axes = [0.0, 0.6, 0.0, 0.0, 0.0, 0.0]
        for _ in range(20):
            publisher.publish_joy(axes)
            executor.spin_once(timeout_sec=0.02)

        # Then: estop (button 4)
        estop_buttons = [0, 0, 0, 0, 1, 0, 0, 0]
        max_spins = 50
        for _ in range(max_spins):
            publisher.publish_joy(axes, buttons=estop_buttons)
            executor.spin_once(timeout_sec=0.02)
            if all(
                abs(collector.values.get(d, 1) or 0) < 0.01
                for d in ["surge", "sway", "heave", "yaw"]
            ):
                break
        else:
            pytest.fail(
                f"Estop did not zero outputs within {max_spins} spins. "
                f"surge={collector.values.get('surge')}"
            )

    finally:
        for node in (teleop, publisher, collector):
            if node is not None:
                node.destroy_node()
