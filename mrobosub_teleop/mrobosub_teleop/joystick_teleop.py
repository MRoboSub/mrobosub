"""
Joystick teleop: button-driven discrete commands with axis scale.

Subscribes to /joy (sensor_msgs/msg/Joy) from joy_node.
Buttons trigger increase/decrease/toggle; axes provide scale factor.
DOFs can toggle between POSE (position) and TWIST (velocity) mode.
"""

import os
from typing import Dict, List

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

from mrobosub_lib import Node

from .joystick_control import Button, JoystickDOF


def _default_params_path() -> str:
    """Default path to joystick_controls.yaml."""
    try:
        pkg_share = get_package_share_directory("mrobosub_teleop")
        return os.path.join(pkg_share, "params", "joystick_controls.yaml")
    except Exception:
        return os.path.join(
            os.path.dirname(__file__), "..", "params", "joystick_controls.yaml"
        )


class JoystickTeleop(Node):
    """
    Button-driven joystick teleop.

    Subscribes:
    - /joy (sensor_msgs/msg/Joy)
    - /pose/{dof} (via JoystickDOF)

    Publishes:
    - /target_twist/{dof}
    - /target_pose/{dof}
    - /output_wrench/{dof} (zeros on estop)
    """

    DOF_ORDER = ["surge", "sway", "heave", "yaw", "roll", "pitch"]

    def __init__(self) -> None:
        super().__init__("joystick_teleop")

        params_path = self.declare_parameter(
            "params_file", _default_params_path()
        ).get_parameter_value().string_value

        with open(params_path, "r") as f:
            params = yaml.safe_load(f)

        axes_config = params["axes"]
        buttons_config = params["buttons"]

        self._dofs: Dict[str, JoystickDOF] = {}
        self._buttons: List[Button] = []
        self._hard_stop = False

        for axis_idx_str, cfg in axes_config.items():
            axis_idx = int(axis_idx_str)
            name = str(cfg["use"])
            scale_t = float(cfg["scale_t"])
            scale_p = float(cfg["scale_p"])
            is_toggleable = name in ("yaw", "heave", "roll", "pitch")
            self._dofs[name] = JoystickDOF(
                axis_idx, name, scale_t, scale_p, is_toggleable, self
            )

        for btn_idx_str, cfg in buttons_config.items():
            btn_idx = int(btn_idx_str)
            cmd_str = str(cfg["use"])
            self._buttons.append(Button(btn_idx, cmd_str))

        self._wrench_pubs = [
            self.create_publisher(Float64, f"/output_wrench/{dof}", 1)
            for dof in self.DOF_ORDER
        ]

        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.create_timer(1.0 / 50, self._loop)

    def _joy_callback(self, msg: Joy) -> None:
        """Process Joy message; act on button rising edges."""
        for button in self._buttons:
            is_pressed = (
                len(msg.buttons) > button.idx and msg.buttons[button.idx] == 1
            )
            button.update(is_pressed)

            if not button.rising_edge:
                continue

            name, command = button.command

            if name == "estop":
                self._hard_stop = True
                continue

            if name == "switch":
                continue

            if name not in self._dofs:
                self.get_logger().warn(f"Button command '{name}' not in DOFs")
                continue

            dof = self._dofs[name]
            axis_val = (
                msg.axes[dof.axis_idx]
                if len(msg.axes) > dof.axis_idx
                else 0.0
            )
            dof.update(command, axis_val)

    def _publish_hard_stop(self) -> None:
        """Publish zeros to output_wrench (bypasses GNC)."""
        for pub in self._wrench_pubs:
            pub.publish(Float64(data=0.0))

    def _reset_target_twist(self) -> None:
        """Publish zeros to target_twist."""
        for dof in self._dofs.values():
            dof.publish_zero_twist()

    def _loop(self) -> None:
        """Timer callback: publish DOF commands or hard stop."""
        if self._hard_stop:
            self._publish_hard_stop()
            self._reset_target_twist()
        else:
            for dof in self._dofs.values():
                dof.publish()


def main() -> None:
    rclpy.init()
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._reset_target_twist()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
