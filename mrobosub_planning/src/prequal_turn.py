from periodic_io import PIO, angle_error
from umrsm import *
from common import *
import prequal_front 
import rospy

class TurnAroundMarker(TurnToYaw):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")
    TimedOut = Outcome.make("TimedOut")

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[float]
    settle_time: Param[float]
    
        
class MovePastMarker(ForwardAndWait):
    Unreached = Outcome.make("MovePastMarker")
    Reached = Outcome.make("TurnToGate")
    
    target_surge_time: Param[float]
    surge_speed: Param[float]
    wait_time: Param[float]
        
       
class TurnToGate(TurnToYaw):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")
    TimedOut = Outcome.make("TimedOut")

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[float]
    settle_time: Param[float]
        
class LeaveMarker(ForwardAndWait):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")
    
    target_surge_time: Param[float]
    surge_speed: Param[float]
    wait_time: Param[float]
    
class ReturnToGate(ForwardAndWait):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")
    
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
