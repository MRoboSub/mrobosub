#!/usr/bin/env python

from py_compile import main
import rclpy
from std_msgs.msg import Float64

from mrobosub_lib import Node

from typing import Optional, Final
from dataclasses import dataclass, field

from math import degrees


@dataclass
class DOF:
    name: str
    short_name: str = field(init=False)
    has_pose: bool
    is_angle: bool

    def __post_init__(self):
        self.name = self.name.lower()
        self.short_name = self.name[:2]

    def __hash__(self):
        return hash(self.name)

    def print_usage(self):
        print(f"{self.name} usage:")
        print(self.name, end="")
        if self.has_pose:
            print(" [twist|pose]", end="")
        if self.is_angle:
            print(" [radians|degrees]", end="")
        print(" value")


# heave is twist and pose
# surge, sway is twist
# yaw, roll, pitch have twist, pose, radians, and degrees
ALL_DOFS_LIST = [
    DOF("heave", True, False),
    DOF("surge", False, False),
    DOF("sway", False, False),
    DOF("yaw", True, True),
    DOF("roll", True, True),
    DOF("pitch", True, True),
]
ALL_DOFS = {dof.short_name: dof for dof in ALL_DOFS_LIST}


class DOFTeleop(Node):
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
    """

    def __init__(self):
        super().__init__("manual_dof_teleop")

        self.dof_publishers = {}
        for dof in ALL_DOFS.values():
            dof_pubs = self.dof_publishers[dof.short_name] = {}
            dof_pubs["twist"] = self.create_publisher(
                Float64, f"/target_twist/{dof.name}", qos_profile=1
            )
            if dof.has_pose:
                dof_pubs["pose"] = self.create_publisher(
                    Float64, f"/target_pose/{dof.name}", qos_profile=1
                )

    def run(self):
        while True:
            rclpy.spin_once(self, timeout_sec=0)
            command = input(
                "Enter a command: dof <twist|pose> <radians|degrees> value\n"
            )
            args = command.split(" ")
            requested_dof = args[0][:2].lower()

            # handle quits
            if requested_dof == "" or requested_dof.startswith("q"):
                print("Quitting!")
                return

            # get dof
            try:
                requested_dof = ALL_DOFS[requested_dof]
            except KeyError:
                print(f'Unknown dof "{requested_dof}"')
                print(
                    "Acceptable options are:",
                    *[dof.name for dof in ALL_DOFS.values()],
                    sep="\n",
                )
                continue
            args = args[1:]
            if len(args) < 1:
                requested_dof.print_usage()
                continue

            value = 0.0

            # handle twist|pose
            mode = "twist"
            if requested_dof.has_pose:
                if args[0].lower().startswith("p"):
                    mode = "pose"
                args = args[1:]
                if len(args) < 1:
                    requested_dof.print_usage()
                    continue
                value = args[0]

            # handle radians|degrees
            if requested_dof.is_angle:
                if len(args) < 2:
                    requested_dof.print_usage()
                    continue
                if args[0].lower().startswith("r"):
                    try:
                        value = degrees(float(args[1]))
                    except ValueError:
                        print(f'Invalid value "{args[1]}"! Must be float-like!')
                        requested_dof.print_usage()
                        continue
                value = args[1]

            # get value
            try:
                requested_target = float(value)
            except ValueError as e:
                print(f'Invalid value "{value}"! Must be float-like!')
                requested_dof.print_usage()
                continue

            # publish
            print(
                f"running command: {requested_dof.name} {mode} {requested_target:5.2f}"
            )
            self.dof_publishers[requested_dof.short_name][mode].publish(
                Float64(data=requested_target)
            )

    def cleanup(self):
        print("Quitting!")
        for dof in self.dof_publishers:
            dof["twist"].publish(0)


def main():
    rclpy.init()

    try:
        DOFTeleop().run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error occurred: {e}")


if __name__ == "__main__":
    main()
