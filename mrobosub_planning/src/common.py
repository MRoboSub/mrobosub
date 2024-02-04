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
    
    target_heave: Param[float]
    heave_threshold: Param[float]
    timeout: Param[int]
    yaw_threshold: Param[float]
    target_yaw: Param[float]
    
    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_heave(self.target_heave)
        PIO.set_target_pose_yaw(self.target_yaw)

        if (PIO.is_heave_within_threshold(self.heave_threshold) and 
                PIO.is_yaw_within_threshold(self.yaw_threshold)):
            return self.Submerged()
        else:
            return self.Unreached()

class Stop(State):
    Surfaced = Outcome.make("Surface")
    Submerged = Outcome.make("Submerged")

    def initialize(self, prev_outcome: Outcome) -> None:
        PIO.set_target_twist_heave(0)
        PIO.set_target_twist_yaw(0)
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_roll(0)
        PIO.set_target_twist_sway(0)
        self.rate = rospy.Rate(50)
    
    def handle(self)->Outcome:
        for _ in range(20):
            PIO.set_target_twist_heave(0)
            PIO.set_target_twist_yaw(0)
            PIO.set_target_twist_surge(0)
            PIO.set_target_twist_roll(0)
            PIO.set_target_twist_sway(0)
            self.rate.sleep()
        return self.Submerged()

Surface = Stop
