"""
Joystick teleop: continuous axis (direct stick) control with button actions.
Buttons trigger on rising edge of press (right when pressed down)
"""

import os
from collections.abc import Callable

import rclpy
import yaml
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from rclpy.parameter import Parameter

from mrobosub_lib import Node, Param

from .joystick_button import Button, ButtonAction

def apply_deadzone(value: float, deadzone: float) -> float:
    """Zero small stick deflections so the sub does not drift."""
    if abs(value) <= deadzone:
        return 0.0
    return value


class JoystickTeleop(Node):
    """
    Subscribes: /joy
    Publishes: /target_twist/{dof}

    Trigger buttons; map axes to twist
    """

    def __init__(self) -> None:
        super().__init__("joystick_teleop")
        
        params = [Param('deadzone', Parameter.Type.DOUBLE, "deadzone where continuous joystick inputs should just be considered 0")]

        self.declare_params(params)

        self.dofs = ["surge", "sway", "heave", "yaw", "roll", "pitch"]

        # all /joy inputs are normalized from -1 to 1; so whatever we write in scale will be multiplied by this value
        # 0,1=left stick 2,3=right stick 4,5=triggers - i think; not too sure about these
        self.mappings: dict[str, dict[str, float]] = {
            "surge": {"axis": 1, "scale": 1.0},
            "sway": {"axis": 0, "scale": 1.0},
            "heave": {"axis": 3, "scale": 1.0},
            "yaw": {"axis": 2, "scale": 1.0},
            "roll": {"axis": 6, "scale": 0.4},
            "pitch": {"axis": 7, "scale": 0.4},
        }

        # create map of publishers for twist values
        self.twist_pubs = {
            dof: self.create_publisher(Float64, f"/target_twist/{dof}", 1)
            for dof in self.dofs
        }

        # Supported actions: estop (hold to zero all outputs)
        self.buttons: list[Button] = [Button(0, "estop"), Button(1, "zero_state")]

        self.zero_imu_client = self.create_client(Trigger, "localization/zero_state")

        self.button_handlers: dict[ButtonAction, Callable[[], None]] = {
            ButtonAction.ESTOP: self.reset_target_twist,
            ButtonAction.ZERO_STATE: self.zero_imu_pose,
        }

        self.create_subscription(Joy, "/joy", self.joy_callback, 10)

    def joy_callback(self, msg: Joy) -> None:
        pass
        # # self.get_logger().warn("joy data")
        # for button in self.buttons:
        #     is_pressed = len(msg.buttons) > button.idx and msg.buttons[button.idx] == 1
        #     button.update(is_pressed)

        # # One-shot actions on rising edge; first match wins, then skip twist publish this tick.
        # for button in self.buttons:
        #     if not button.just_pressed:
        #         continue
        #     handler = self.button_handlers.get(button.action)
        #     if handler is not None:
        #         handler()
        #         return

        # # No button consumed this tick — publish stick-driven twist.
        # for dof, mapping in self.mappings.items():
        #     axis_idx = mapping["axis"]
        #     scale = mapping["scale"]
        #     raw = msg.axes[axis_idx] if len(msg.axes) > axis_idx else 0.0
        #     deadzoned = apply_deadzone(raw, self.deadzone)
        #     value = deadzoned ** 3 * scale  # cubic curve: finer control near center
        #     self.twist_pubs[dof].publish(Float64(data=value))

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
    node = JoystickTeleop()
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
