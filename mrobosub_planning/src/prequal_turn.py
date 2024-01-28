from typing import NamedTuple
from periodic_io import PIO, angle_error
from umrsm import *
from common import *
import prequal_front 
import rospy
from umrsm import TransitionMap

class TurnAroundMarker(TurnToYaw):
    class Unreached(NamedTuple): pass
    class Reached(NamedTuple): pass
    class TimedOut(NamedTuple): pass

    target_yaw: float = 90
    yaw_threshold: float = 2 
    timeout: float = 10
    settle_time: float = 1

    def handle_reached(self) -> NamedTuple:
        return self.Reached()

    def handle_unreached(self) -> NamedTuple:
        return self.Unreached()

    def handle_once_timedout(self) -> NamedTuple:
        return self.TimedOut()
    
        
class MovePastMarker(ForwardAndWait):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    
    target_heave: float = 1.2
    target_surge_time: float = 8
    surge_speed: float = 0.2
    wait_time: float = 1

    def handle_reached(self) -> NamedTuple:
        return self.Reached()
    
    def handle_unreached(self) -> NamedTuple:
        return self.Unreached()

class TurnToGate(TurnToYaw):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    target_yaw: float = -175
    yaw_threshold: float = 2
    timeout: float = 10
    settle_time: float = 1

    def handle_unreached(self) -> NamedTuple:
        return self.Unreached()

    def handle_reached(self) -> NamedTuple:
        return self.Reached()

    def handle_once_timedout(self) -> NamedTuple:
        return self.TimedOut()
        
class LeaveMarker(ForwardAndWait):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    
    target_heave: float = 1.2
    target_surge_time: float = 10
    surge_speed: float = 0.2
    wait_time: float = 1

    def handle_reached(self) -> NamedTuple:
        return self.Reached()

    def handle_unreached(self) -> NamedTuple:
        return self.Unreached()
    
class ReturnToGate(ForwardAndWait):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    
    target_heave: float = 1.9
    target_surge_time: float = 224
    surge_speed: float = 0.2
    wait_time: float = 1

    def handle_unreached(self) -> NamedTuple:
        return self.Unreached()

    def handle_reached(self) -> NamedTuple:
        return self.Reached()

transitions: TransitionMap = prequal_front.transitions.copy()
transitions.update({
    prequal_front.ApproachMarker.Reached: TurnAroundMarker,

    TurnAroundMarker.Unreached: TurnAroundMarker,
    TurnAroundMarker.Reached: MovePastMarker,
    TurnAroundMarker.TimedOut: MovePastMarker,

    MovePastMarker.Unreached: MovePastMarker,
    MovePastMarker.Reached: TurnToGate,

    TurnToGate.Unreached: TurnToGate,
    TurnToGate.Reached: LeaveMarker,
    TurnToGate.TimedOut: LeaveMarker,

    LeaveMarker.Unreached: LeaveMarker,
    LeaveMarker.Reached: ReturnToGate,

    ReturnToGate.Unreached: ReturnToGate,
    ReturnToGate.Reached: Stop,
})
