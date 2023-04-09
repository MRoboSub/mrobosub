from periodic_io import PIO, angle_error
from umrsm import *
from common import *
import prequal_front 
import rospy

class TurnAroundMarker(TimedState):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")
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
    
        
class MovePastMarker(State):
    Unreached = Outcome.make("MovePastMarker")
    Reached = Outcome.make("TurnToGate")
    
    target_surge_time: Param[float]
    surge_speed: Param[float]

    def initialize(self, prev_outcome: Outcome)-> None:
        self.start_time = rospy.get_time()
        
    def handle(self)->Outcome:
        PIO.set_target_twist_surge(self.surge_speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            PIO.set_target_twist_surge(0)
            return self.Reached()
        else:
            return self.Unreached()
        
       
class TurnToGate(TimedState):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")
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
        
class ReturnToGate(State):
    Unreached = Outcome.make("ReturnToGate")
    Reached = Outcome.make("Stop")
    
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


transitions = prequal_front.transitions
transitions.update({
    prequal_front.ApproachMarker.Reached: TurnAroundMarker,

    TurnAroundMarker.Unreached: TurnAroundMarker,
    TurnAroundMarker.Reached: MovePastMarker,
    TurnAroundMarker.TimedOut: MovePastMarker,

    MovePastMarker.Unreached: MovePastMarker,
    MovePastMarker.Reached: TurnToGate,

    TurnToGate.Unreached: TurnToGate,
    TurnToGate.Reached: ReturnToGate,
    TurnToGate.TimedOut: ReturnToGate,

    ReturnToGate.Unreached: ReturnToGate,
    ReturnToGate.Reached: Stop,
})
