from mrobosub_planning.src.common import Surface
from umrsm import *
import prequal_front
from periodic_io import PIO
import rospy

class StrafeAroundMarker(State):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")

    target_sway_time: Param[float]
    sway_speed: Param[float]
        
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_sway(self.sway_speed)
        if rospy.get_time() - self.start_time >= self.target_sway_time:
            PIO.set_target_twist_sway(0)
            return self.Reached()
        else:
            return self.Unreached()
        
            
class ReturnGate(State):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")

    target_surge_time: Param[float]
    speed: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(-self.speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            PIO.set_target_twist_surge(0)
            return self.Reached()
        else:
            return self.Unreached()

transitions = prequal_front.transitions | {
    prequal_front.ApproachMarker.Reached: StrafeAroundMarker,

    StrafeAroundMarker.Unreached: StrafeAroundMarker,
    StrafeAroundMarker.Reached: ReturnGate,

    ReturnGate.Unreached: ReturnGate,
    ReturnGate.ReturnGate: Surface,
}
