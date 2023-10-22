from periodic_io import PIO, angle_error
from umrsm import *
from common import *
import prequal_front 
import rospy

class TurnAroundMarker(TurnToYaw):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[float]
    settle_time: Param[float]
    
        
class MovePastMarker(ForwardAndWait):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    
    target_surge_time: Param[float]
    surge_speed: Param[float]
    wait_time: Param[float]

class TurnToGate(TurnToYaw):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[float]
    settle_time: Param[float]
        
class LeaveMarker(ForwardAndWait):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    
    target_surge_time: Param[float]
    surge_speed: Param[float]
    wait_time: Param[float]
    
class ReturnToGate(ForwardAndWait):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    
    target_surge_time: Param[float]
    surge_speed: Param[float]
    wait_time: Param[float]

transitions = prequal_front.transitions
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
