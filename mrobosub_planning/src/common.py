from mrobosub_planning.src.umrsm import Outcome
from umrsm import *
from periodic_io import PIO
import rospy
from typing import NamedTuple

class Start(State):
    class Complete(NamedTuple):
        pass
    
    def initialize(self, prev_outcome) -> None:
        pass
    
    def handle(self):
        return self.Complete()

class Submerge(TimedState):
    class Unreached(NamedTuple):
        pass
    class Submerged(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass
    
    target_heave: Param[float]
    heave_threshold: Param[float]
    m_timeout: Param[float]
    yaw_threshold: Param[float]
    target_yaw: Param[float]
    
    def handle_if_not_timedout(self):
        PIO.set_target_pose_heave(self.target_heave)
        PIO.set_target_pose_yaw(self.target_yaw)

        if (PIO.is_heave_within_threshold(self.heave_threshold) and 
                PIO.is_yaw_within_threshold(self.yaw_threshold)):
            return self.Submerged()
        else:
            return self.Unreached()
    
    def handle_once_timedout(self):
        return self.TimedOut()
    
    def timeout(self) -> float:
        return self.m_timeout

class Stop(State):
    class Surfaced(NamedTuple):
        pass
    class Submerged(NamedTuple):
        pass

    def initialize(self, prev_outcome) -> None:
        PIO.set_target_twist_heave(0)
        PIO.set_target_twist_yaw(0)
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_roll(0)
        PIO.set_target_twist_sway(0)
        self.rate = rospy.Rate(50)
    
    def handle(self):
        for _ in range(20):
            PIO.set_target_twist_heave(0)
            PIO.set_target_twist_yaw(0)
            PIO.set_target_twist_surge(0)
            PIO.set_target_twist_roll(0)
            PIO.set_target_twist_sway(0)
            self.rate.sleep()
        return self.Submerged()

Surface = Stop
