#!/usr/bin/env python
from typing import NamedTuple, Dict, Type
from periodic_io import PIO, angle_error
from umrsm import *
from common import *
import rospy

class AlignGate(TimedState):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    m_timeout: Param[float]

    def handle_if_not_timedout(self):
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Reached()
        else:
            return self.Unreached()
    
    def handle_once_timedout(self):
        return self.TimedOut()

    def timeout(self) -> float:
        return self.m_timeout
        
class ApproachGate(ForwardAndWait):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
 
    m_target_surge_time: Param[float]
    m_surge_speed: Param[float]
    m_wait_time: Param[float]

    def target_surge_time(self) -> float:
        return self.m_target_surge_time
    
    def surge_speed(self) -> float:
        return self.m_surge_speed
    
    def wait_time(self) -> float:
        return self.m_wait_time
    
    # TODO
    def target_heave(self) -> float:
        return -1.0

    def reached_state(self):
        return self.Reached()

    def unreached_state(self):
        return self.Unreached()

        
# Concerns: -drift during submerge, poor movement on sway &surge axes, drift during surge.

class ApproachMarker(ForwardAndWait):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass

    m_target_surge_time: Param[float]
    m_surge_speed: Param[float]
    m_wait_time: Param[float]

    def target_surge_time(self) -> float:
        return self.m_target_surge_time
    
    def surge_speed(self) -> float:
        return self.m_surge_speed
    
    def wait_time(self) -> float:
        return self.m_wait_time
    
    # TODO
    def target_heave(self) -> float:
        return -1.0

    def reached_state(self):
        return self.Reached()

    def unreached_state(self):
        return self.Unreached()

transitions: Dict[Type[NamedTuple], Type[State]] = {
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
