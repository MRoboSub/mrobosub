#!/usr/bin/env python

from base_teleop import AngleBaseTeleop

from math import degrees


class PitchTeleop(AngleBaseTeleop):
    def __init__(self):
        super().__init__("pitch")


if __name__ == "__main__":
    PitchTeleop().run()
