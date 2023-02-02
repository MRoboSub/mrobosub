#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

from math import degrees

class TeleopNode(Node):
    """
    Publishers
    - /pose/{type_}
    """

    def __init__(self, type_: str):
        super().__init__(f"{type_}_teleop")

        self.twist_pub = rospy.Publisher(f"/target_twist/{type_}", Float64, queue_size=1)
        self.pose_pub = rospy.Publisher(f"/target_pose/{type_}", Float64, queue_size=1)

    def run(self): 
        while True:
            input_node = input("Which input would you like: \n[T]wist \n[P]ose \n[Q]uit \n").upper()

            curr_node = None
            if input_node.startswith("T"):
                curr_node = self.twist_pub
            elif input_node.startswith("P"):
                curr_node = self.pose_pub
            elif input_node.startswith("Q"):
                return
            else:
                print("Invalid input mode\n")
                continue

            self.config()

            while True:
                request = self.get_input()
                if request is None:
                    break
                desired_target = float(request)

                try:
                    curr_node.publish(desired_target)
                except:
                    break

    def cleanup(self):
        self.twist_pub.publish(0)

    def config(self):
        ...

    def get_input(self) -> Optional[float]:
        try:
            return float(input("Input your desired value (or change to change mode):\n"))
        except ValueError:
            return None


class AngleBaseTeleop(TeleopNode):
    def __init__(self, type_: str):
        super().__init__(type_)

    def config(self):
        while True:
            input_type = input("Which angle type would you like: \n[R]adians\n[D]egrees\n").upper()
            if input_type.startswith("R"):
                self.angle_convert = degrees
            elif input_type.startswith("D"):
                self.angle_convert = lambda x: x
            else:
                print("Invalid angle type\n")
                continue
            return

    def get_input(self) -> Optional[float]:
        value = super().get_input()
        if value is None:
            return None
        return self.angle_convert(value)
