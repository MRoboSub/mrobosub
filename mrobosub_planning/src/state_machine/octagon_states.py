import rospy
from umrsm import State
from abstract_states import TurnToYaw, TimedState
from periodic_io import PIO
from typing import Dict, Optional, Tuple, Union, NamedTuple


class TurnToOctagon(TurnToYaw):
    class Aligned(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    target_yaw = 30.0
    yaw_threshold = 2.0
    settle_time = 1.0
    timeout = 10.0

    def handle_reached(self) -> Aligned:
        return self.Aligned()

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class GoToOctagon(TimedState):
    class Reached(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    timeout: float = 20.0
    yaw_angle: float = 30.0
    surge_speed: float = 0.15

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)

    def handle_if_not_timedout(self) -> None:
        PIO.set_target_pose_yaw(self.yaw_angle)
        PIO.set_target_twist_surge(self.surge_speed)

    def handle_once_timedout(self) -> Reached:
        PIO.set_target_twist_surge(0)
        return self.Reached()
