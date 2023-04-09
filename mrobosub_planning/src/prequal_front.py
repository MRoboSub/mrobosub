#!/usr/bin/env python
from periodic_io import PIO, angle_error
from umrsm import *
from common import *
import rospy

# TODO: switch a bunch of these states to use TimedState rather than manually managing the timeout

class AlignGate(TimedState):
    Unaligned = Outcome.make('Unaligned')
    ReachedAngle = Outcome.make('ReachedAngle')
    TimedOut = Outcome.make("TimedOut")

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[int]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.ReachedAngle()
        else:
            return self.Unaligned()
    
        
class ApproachGate(State):
    Unreached = Outcome.make('Unreached')
    Reached = Outcome.make("Reached")
 
    target_surge_time: Param[float]
    surge_speed: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(self.surge_speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            PIO.set_target_twist_surge(0)
            return self.Reached()
        else:
            return self.Unreached()

        
# Concerns: -drift during submerge, poor movement on sway &surge axes, drift during surge.

class ApproachMarker(State):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")

    target_surge_time: Param[float]
    surge_speed: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(self.surge_speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            PIO.set_target_twist_surge(0)
            return self.Reached()
        else:
            return self.Unreached()

transitions = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.Unaligned: AlignGate,
    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.Unreached: ApproachGate,
    ApproachGate.Reached: ApproachMarker,

    ApproachMarker.Unreached: ApproachMarker,
    
    #Surface.Unreached: Surface,
    #Surface.Reached: Stop
}
