#!/usr/bin/env python
from mrobosub_planning.src.umrsm import Outcome
from periodic_io import PIO, angle_error
from umrsm import *
from common import *
import rospy

class AlignGate(TimedState):
    Unreached = Outcome.make('Unreached')
    Reached = Outcome.make('Reached')
    TimedOut = Outcome.make("TimedOut")

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[int]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Reached()
        else:
            return self.Unreached()
    
    def handle_once_timedout(self) -> Outcome:
        return self.TimedOut()
        
class ApproachGate(ForwardAndWait):
    Unreached = Outcome.make('Unreached')
    Reached = Outcome.make("Reached")
 
    target_surge_time: Param[float]
    surge_speed: Param[float]
    wait_time: Param[float]

        
# Concerns: -drift during submerge, poor movement on sway &surge axes, drift during surge.

class ApproachMarker(ForwardAndWait):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")

    target_surge_time: Param[float]
    surge_speed: Param[float]
    wait_time: Param[float]

transitions = {
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
