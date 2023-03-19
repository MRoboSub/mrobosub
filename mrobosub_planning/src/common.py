from umrsm import *
from periodic_io import PIO
import rospy

class Start(State):
    Complete = Outcome.make("Complete")
    
    def initialize(self, prev_outcome: Outcome) -> None:
        pass
    
    def handle(self) -> Outcome:
        return self.Complete()

class Submerge(TimedState):
    Unreached = Outcome.make("Unreached")
    Submerged = Outcome.make("Submerged")
    TimedOut = Outcome.make("TimedOut")
    
    target_depth: Param[float]
    timeout: Param[int]
    
    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_heave(self.target_depth)

        if PIO.Pose.heave > self.target_depth:
            return self.Submerged()
        else:
            return self.Unreached()
    

class Surface(State):
    # TODO
    pass

class Stop(State):
    Finish = Outcome.make("Finish")

    def initialize(self, prev_outcome: Outcome) -> None:
        PIO.forward = 0
        PIO.lateral = 0
        PIO.target_depth = 0
        PIO.set_override_heading(0)
    
    def handle(self)->Outcome:
        return self.Finish()
