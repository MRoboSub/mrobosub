#!/usr/bin/env python

from base_teleop import AngleBaseTeleop

from math import degrees


class RollTeleop(AngleBaseTeleop):
    def __init__(self):
        super().__init__("roll")


if __name__ == "__main__":
    RollTeleop().run()
