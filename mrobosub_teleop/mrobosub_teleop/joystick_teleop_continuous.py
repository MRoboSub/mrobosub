"""
Joystick teleop: continuous axis (direct stick) control with button actions.

Push stick = move, release = stop.
Buttons trigger discrete actions (estop, etc.) while held.
Subscribes to /joy (sensor_msgs/msg/Joy) from joy_node.
"""

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from mrobosub_lib import Node

from .joystick_button import Button


def _default_params_path() -> str:
    """Default path to joystick_continuous.yaml."""
    try:
        pkg_share = get_package_share_directory("mrobosub_teleop")
        return os.path.join(pkg_share, "params", "joystick_continuous.yaml")
    except Exception:
        return os.path.join(
            os.path.dirname(__file__), "..", "params", "joystick_continuous.yaml"
        )


def _apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) <= deadzone:
        return 0.0
    return value


class JoystickTeleopContinuous(Node):
    """
    Continuous joystick teleop: axes map directly to target_twist.

    Buttons trigger discrete actions on rising edge only (press, not hold).
    Supported button actions: estop (zeros all outputs on press).

    Subscribes: /joy
    Publishes: /target_twist/{dof}
    """

    def __init__(self) -> None:
        super().__init__("joystick_teleop_continuous")

        params_path = self.declare_parameter(
            "params_file", _default_params_path()
        ).get_parameter_value().string_value

        with open(params_path, "r") as f:
            params = yaml.safe_load(f)

        self._deadzone = float(params.get("deadzone", 0.1))

        self._mappings: dict[str, dict[str, float]] = {}
        for dof in ["surge", "sway", "heave", "yaw", "roll", "pitch"]:
            cfg = params.get(dof)
            if cfg:
                self._mappings[dof] = {
                    "axis": int(cfg["axis"]),
                    "scale": float(cfg.get("scale", 0.5)),
                }

        # create map of publishers for twist values
        self._twist_pubs = {
            dof: self.create_publisher(Float64, f"/target_twist/{dof}", 1)
            for dof in self._mappings
        }

        self._buttons: list[Button] = []
        for btn_idx_str, cfg in params.get("buttons", {}).items():
            self._buttons.append(Button(int(btn_idx_str), str(cfg["use"])))

        # needed for zeroing state
        self._zero_imu_client = self.create_client(Trigger, "localization/zero_state")

        self.create_subscription(Joy, "/joy", self._joy_callback, 10)

    def _joy_callback(self, msg: Joy) -> None:
        """Map axes to target_twist; dispatch button actions."""
        for button in self._buttons:
            is_pressed = len(msg.buttons) > button.idx and msg.buttons[button.idx] == 1
            button.update(is_pressed)

        for button in self._buttons:
            if not button.just_pressed:
                continue
            match button.name:
                case "estop":
                    self._reset_target_twist()
                    return
                case "zero_state":
                    self._zero_imu_pose()
                    return
                case _:
                    continue

        # publish twist values for everthing
        for dof, mapping in self._mappings.items():
            axis_idx = mapping["axis"]
            scale = mapping["scale"]
            raw = msg.axes[axis_idx] if len(msg.axes) > axis_idx else 0.0
            deadzoned = _apply_deadzone(raw, self._deadzone)
            value = deadzoned ** 3 * scale # cubic scale for better control
            self._twist_pubs[dof].publish(Float64(data=value))

    def _reset_target_twist(self) -> None:
        """Publish zero to all target_twist topics."""
        for pub in self._twist_pubs.values():
            pub.publish(Float64(data=0.0))

    def _zero_imu_pose(self) -> None:
        """Call localization/zero_state to reset IMU pose offsets."""
        if not self._zero_imu_client.service_is_ready():
            self.get_logger().warn("localization/zero_state service not available")
            return
        self._zero_imu_client.call_async(Trigger.Request())



def main() -> None:
    rclpy.init()
    node = JoystickTeleopContinuous()
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
