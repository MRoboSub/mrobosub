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
    

Surface = Stop

class Stop(State):
    Finish = Outcome.make("Finish")

    def initialize(self, prev_outcome: Outcome) -> None:
        PIO.set_target_pose_heave(0)
        PIO.set_target_twist_yaw(0)
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_roll(0)
        PIO.set_target_twist_sway(0)
    
    def handle(self)->Outcome:
        PIO.set_target_pose_heave(0)
        PIO.set_target_twist_yaw(0)
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_roll(0)
        PIO.set_target_twist_sway(0)
        return self.Finish()
