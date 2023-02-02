#!/usr/bin/env python

from base_teleop import TeleopNode


class HeaveTeleop(TeleopNode):
    def __init__(self):
        super().__init__("heave")

if __name__ == "__main__":
    HeaveTeleop().run()
