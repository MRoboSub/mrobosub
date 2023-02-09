#!/usr/bin/env python

from base_teleop import TeleopNode


class SwayTeleop(TeleopNode):
    def __init__(self):
        super().__init__("sway")

if __name__ == "__main__":
    SwayTeleop().run()
