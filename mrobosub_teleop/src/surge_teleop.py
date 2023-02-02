#!/usr/bin/env python

from base_teleop import TeleopNode


class SurgeTeleop(TeleopNode):
    def __init__(self):
        super().__init__("surge")

if __name__ == "__main__":
    SurgeTeleop().run()
