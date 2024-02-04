#!/usr/bin/env python
from periodic_io import PIO, angle_error
from umrsm import *
from umrsm import TransitionMap
from common import *
from abstract_states import ForwardAndWait
from typing import NamedTuple


class AlignGate(TimedState):
    class Reached(NamedTuple): pass
    class TimedOut(NamedTuple): pass
    
    target_yaw: float = 0.0
    yaw_threshold: float = 2.0
    timeout: float = 10.0

    def handle_if_not_timedout(self) -> Union[Reached, None]:
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Reached()
        return None

    def handle_once_timedout(self):
        return self.TimedOut()


class ApproachGate(ForwardAndWait):
    class Reached(NamedTuple): pass

    target_heave: float = 2 # TODO
    target_surge_time: float = 10.0
    surge_speed: float = 0.2
    wait_time: float = 1.0

    def handle_reached(self) -> Reached:
        return self.Reached()

    def handle_unreached(self) -> None:
        return None


# Concerns: -drift during submerge, poor movement on sway &surge axes, drift during surge.

class ApproachMarker(ForwardAndWait):
    class Reached(NamedTuple): pass

    target_heave: float = 2 # TODO
    target_surge_time: float = 22.0
    surge_speed: float = 0.2
    wait_time: float = 1.0

    def handle_reached(self) -> Reached:
        return self.Reached()

    def handle_unreached(self) -> None:
        return None


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.Reached: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.Reached: ApproachMarker,

    #Surface.Unreached: Surface,
    #Surface.Reached: Stop
}
