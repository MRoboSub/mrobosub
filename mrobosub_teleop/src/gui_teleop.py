#!/usr/bin/env python

import tkinter as tk
from dataclasses import dataclass, field
from math import degrees
from typing import Final, Optional

import rospy
from std_msgs.msg import Float64

from mrobosub_lib.lib import Node, Param


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


ALL_DOFS = [
    DOF("heave", True, False),
    DOF("surge", False, False),
    DOF("sway", False, False),
    DOF("yaw", True, True),
    DOF("roll", True, True),
    DOF("pitch", True, True),
]


class MainGUI(tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


if __name__ == "__main__":
    # root = tk.Tk()

    frame = MainGUI()
    frame.mainloop()
