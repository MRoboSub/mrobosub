#!/usr/bin/env python

from enum import Enum
from typing import List

from matplotlib import axes
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from typing import Dict, Union, Tuple
from mrobosub_lib import Node
import os
from ament_index_python.packages import get_package_share_directory


class ButtonCommand(int, Enum):
    INCREASE = 1
    DECREASE = -1
    TOGGLE = 0


class Button:
    """Store commands tied to buttons"""

    def __init__(self, idx: int, commandstr: str) -> None:
        self.idx = idx
        self.commandstr = commandstr
        self._ispressed = False
        self._isrising_edge = False
        self.name, self._action = self.__split_command(commandstr)

    @staticmethod
    def __split_command(comstr: str) -> Tuple[str, ButtonCommand]:
        """Split command string into (axis, command)"""
        if comstr.endswith(".increase"):
            return (comstr[:-9], ButtonCommand.INCREASE)
        elif comstr.endswith(".decrease"):
            return (comstr[:-9], ButtonCommand.DECREASE)
        elif comstr.endswith(".toggle"):
            return (comstr[:-7], ButtonCommand.TOGGLE)
        # Special commands
        elif comstr.endswith("estop") or comstr.endswith("switch"):
            return (comstr[4:], ButtonCommand.TOGGLE)
        else:
            raise ValueError(f"Button command {comstr} not recognised")

    def update(self, ispress: bool) -> None:
        self._isrising_edge = (not self.pressed) and ispress
        self._ispressed = ispress

    @property
    def rise_edge(self) -> bool:
        return self._isrising_edge

    @property
    def pressed(self) -> bool:
        return self._ispressed

    @property
    def command(self) -> Tuple[str, ButtonCommand]:
        return self.name, self._action


class DOF:
    """Store state of a degree of freedom"""

    class DOFState(int, Enum):
        POSE = 0
        TWIST = 1

    def __init__(
        self,
        id: int,
        name: str,
        scale_t: float,
        scale_p: float,
        istoggleable: bool,
        node: Node,
    ) -> None:
        self.idx = id
        self.name = name

        ## state variables
        self.pos = 0.0  # current position
        self.state = DOF.DOFState.TWIST
        self.scale = 0.0  # current scale
        self.setPoint = 0.0  # target position

        ## configs
        self.scale_t = scale_t
        self.scale_p = scale_p
        self.istogglable = istoggleable

        self.twist_pub = node.create_publisher(
            Float64, f"/target_twist/{name}", qos_profile=1
        )
        self.pose_pub = node.create_publisher(
            Float64, f"/target_pose/{name}", qos_profile=1
        )
        self.pose_sub = node.create_subscription(
            Float64, f"/pose/{name}", self.pose_callback, qos_profile=1
        )

    def pose_callback(self, msg: Float64) -> None:
        self.pos = msg.data

    def update(self, command: ButtonCommand, scale: float) -> None:
        self.state ^= int(
            not bool(abs(command)) and self.istogglable
        )  # toggle if command is TOGGLE && istoggleable
        self.scale = (
            self.scale_t * self.state + self.scale_p * (1 - self.state)
        ) * scale  # set scale based on state
        self.setPoint = self.pos + self.scale_p * scale

    def publish(self) -> None:
        if self.state == DOF.DOFState.POSE:
            self.pose_pub.publish(Float64(data=self.setPoint))
        elif self.state == DOF.DOFState.TWIST:
            self.twist_pub.publish(Float64(data=self.scale))


class Joystick_teleop(Node):
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
    - /pose/roll
    - /pose/pitch
    """

    # axes : Dict[str, Dict[str, Union[str,float]]]
    # buttons : Dict[str, Dict[str, str]]

    def __init__(self) -> None:
        super().__init__("joystick_teleopn")
        # load params

        paramsfile = paramsfile = (
            "/home/ubuntu/ros2_ws/src/mrobosub_teleop/params/joystick_controls_b2a.yaml"
        )
        # load directly from params folder, instead of ros2 param server (do not support recursive dict)

        with open(paramsfile, "r") as f:
            import yaml

            params = yaml.safe_load(f)
        self.axes = params["axes"]
        self.buttons = params["buttons"]

        print(self.axes)
        print(self.buttons)

        self.input_subscriber = self.create_subscription(
            Joy, "/joy", self.joystick_callback, qos_profile=1
        )
        self.wrench_pubs = [
            self.create_publisher(Float64, f"/output_wrench/{axis}", qos_profile=1)
            for axis in ["sway", "surge", "heave", "yaw", "roll", "pitch"]
        ]
        # Globals
        self.buttonsInstance: List[Button] = []
        self.DOFInstance: Dict[str, DOF] = {}
        self.stateMachineMode = False
        self.timer = self.create_timer(1.0 / 50, self.loop)
        for i in range(6):
            if DOFConfig := self.axes.get(str(i)):
                name = str(DOFConfig["use"])
                self.DOFInstance[name] = DOF(
                    i,
                    name,
                    float(DOFConfig["scale_t"]),
                    float(DOFConfig["scale_p"]),
                    name in ["yaw", "heave", "roll", "pitch"],
                    self,
                )  # toggleable DOFs
            else:
                raise ValueError(f"Axis {i} not configured in params")
        for i in range(10):
            if ButtonConfig := self.buttons.get(str(i)):
                self.buttonsInstance.append(Button(i, str(ButtonConfig["use"])))
            else:
                raise ValueError(f"Button {i} not configured in params")

    def joystick_callback(self, msg: Joy):
        for button in self.buttonsInstance:
            button.update(msg.buttons[button.idx] == 1)
            if button.rise_edge:
                name, command = button.command
                if name == "estop":
                    self.hard_stop()
                    return
                elif name == "switch":
                    self.stateMachineMode ^= True
                else:
                    if tar := self.DOFInstance.get(name):
                        tar.update(command, msg.axes[tar.idx])
                    else:
                        raise ValueError(f"Button command {name} not recognised")

    def hard_stop(self):
        for pub in self.wrench_pubs:
            pub.publish(Float64(data=0.0))
        return

    def loop(self):
        if not self.stateMachineMode:
            for dof in self.DOFInstance.values():
                dof.publish()
            else:
                self.hard_stop()


def main():
    rclpy.init()
    node = Joystick_teleop()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
