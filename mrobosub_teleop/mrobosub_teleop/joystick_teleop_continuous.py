"""
Joystick teleop: continuous axis (direct stick) control with button actions.
Buttons trigger on rising edge of press (right when pressed down)
"""

import os
from collections.abc import Callable

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from mrobosub_lib import Node

from .joystick_button import Button, ButtonAction


def default_params_path() -> str:
    """Installed share path, or source-tree fallback for dev without install."""
    try:
        pkg_share = get_package_share_directory("mrobosub_teleop")
        return os.path.join(pkg_share, "params", "joystick_continuous.yaml")
    except Exception:
        return os.path.join(
            os.path.dirname(__file__), "..", "params", "joystick_continuous.yaml"
        )


def apply_deadzone(value: float, deadzone: float) -> float:
    """Zero small stick deflections so the sub does not drift."""
    if abs(value) <= deadzone:
        return 0.0
    return value


class JoystickTeleopContinuous(Node):
    """
    Subscribes: /joy
    Publishes: /target_twist/{dof}

    Trigger buttons; map axes to twist
    """

    def __init__(self) -> None:
        super().__init__("joystick_teleop_continuous")

        params_path = self.declare_parameter(
            "params_file", default_params_path()
        ).get_parameter_value().string_value

        with open(params_path, "r") as f:
            params = yaml.safe_load(f)

        self.deadzone = float(params.get("deadzone", 0.1))


        self.mappings: dict[str, dict[str, float]] = {}
        for dof in ["surge", "sway", "heave", "yaw", "roll", "pitch"]:
            cfg = params.get(dof)
            if cfg:
                self.mappings[dof] = {
                    "axis": int(cfg["axis"]),
                    "scale": float(cfg.get("scale", 0.5)),
                }

        # create map of publishers for twist values
        self.twist_pubs = {
            dof: self.create_publisher(Float64, f"/target_twist/{dof}", 1)
            for dof in self.mappings
        }

        self.buttons: list[Button] = []
        for btn_idx_str, cfg in params.get("buttons", {}).items():
            self.buttons.append(Button(int(btn_idx_str), str(cfg["use"])))

        self.zero_imu_client = self.create_client(Trigger, "localization/zero_state")

        # Dispatch by enum: extend ButtonAction + YAML ``use`` strings, then add a handler here.
        self.button_handlers: dict[ButtonAction, Callable[[], None]] = {
            ButtonAction.ESTOP: self.reset_target_twist,
            ButtonAction.ZERO_STATE: self.zero_imu_pose,
        }

        self.create_subscription(Joy, "/joy", self.joy_callback, 10)

    def joy_callback(self, msg: Joy) -> None:
        for button in self.buttons:
            is_pressed = len(msg.buttons) > button.idx and msg.buttons[button.idx] == 1
            button.update(is_pressed)

        # One-shot actions on rising edge; first match wins, then skip twist publish this tick.
        for button in self.buttons:
            if not button.just_pressed:
                continue
            handler = self.button_handlers.get(button.action)
            if handler is not None:
                handler()
                return

        # No button consumed this tick — publish stick-driven twist.
        for dof, mapping in self.mappings.items():
            axis_idx = mapping["axis"]
            scale = mapping["scale"]
            raw = msg.axes[axis_idx] if len(msg.axes) > axis_idx else 0.0
            deadzoned = apply_deadzone(raw, self.deadzone)
            value = deadzoned ** 3 * scale  # cubic curve: finer control near center
            self.twist_pubs[dof].publish(Float64(data=value))

    def reset_target_twist(self) -> None:
        for pub in self.twist_pubs.values():
            pub.publish(Float64(data=0.0))

    def zero_imu_pose(self) -> None:
        if not self.zero_imu_client.service_is_ready():
            self.get_logger().warn("localization/zero_state service not available")
            return
        self.zero_imu_client.call_async(Trigger.Request())



def main() -> None:
    rclpy.init()
    node = JoystickTeleopContinuous()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_target_twist()  # stop sending twist after exit / SIGINT
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
