#!/usr/bin/env python

from base_teleop import AngleBaseTeleop

from math import degrees


class YawTeleop(AngleBaseTeleop):
    def __init__(self):
        super().__init__("yaw")


if __name__ == "__main__":
    YawTeleop().run()
