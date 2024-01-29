#!/usr/bin/env python
from periodic_io import PIO, angle_error
from umrsm import *
from common import *
import rospy
from umrsm import TransitionMap


class AlignGate(TimedState):
    class Unreached(Outcome): pass
    class Reached(Outcome): pass
    class TimedOut(Outcome): pass
    
    target_yaw: float = 0.0
    yaw_threshold: float = 2.0
    timeout: float = 10.0

    def handle_if_not_timedout(self):
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Reached()
        else:
            return self.Unreached()

    def handle_once_timedout(self):
        return self.TimedOut()


class ApproachGate(ForwardAndWait):
    class Unreached(Outcome): pass
    class Reached(Outcome): pass

    target_heave: float = 2 # TODO
    target_surge_time: float = 10.0
    surge_speed: float = 0.2
    wait_time: float = 1.0

    def handle_reached(self) -> Outcome:
        return self.Reached()

    def handle_unreached(self) -> Outcome:
        return self.Unreached()


# Concerns: -drift during submerge, poor movement on sway &surge axes, drift during surge.

class ApproachMarker(ForwardAndWait):
    class Unreached(Outcome): pass
    class Reached(Outcome): pass

    target_heave: float = 2 # TODO
    target_surge_time: float = 22.0
    surge_speed: float = 0.2
    wait_time: float = 1.0

    def handle_reached(self) -> Outcome:
        return self.Reached()

    def handle_unreached(self) -> Outcome:
        return self.Unreached()


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.Unreached: AlignGate,
    AlignGate.Reached: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.Unreached: ApproachGate,
    ApproachGate.Reached: ApproachMarker,

    ApproachMarker.Unreached: ApproachMarker,
    
    #Surface.Unreached: Surface,
    #Surface.Reached: Stop
}
