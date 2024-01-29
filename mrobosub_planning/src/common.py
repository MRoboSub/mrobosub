from umrsm import *
from periodic_io import PIO
import rospy

class Start(State):
    class Complete(Outcome): pass
    
    def handle(self):
        return self.Complete()

class Submerge(TimedState):
    class Unreached(Outcome): pass
    class Submerged(Outcome): pass
    class TimedOut(Outcome): pass
    
    target_heave: float = 0.35
    heave_threshold: float = 0.1
    timeout: float = 15
    yaw_threshold: float = 0
    target_yaw: float = 2
    
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

class Stop(State):
    class Surfaced(Outcome): pass
    class Submerged(Outcome): pass

    def __init__(self, prev_outcome: Outcome):
        super().__init__(prev_outcome)
        PIO.reset_target_twist()
        self.rate = rospy.Rate(50)
    
    def handle(self):
        for _ in range(20):
            PIO.reset_target_twist()
            self.rate.sleep()
        return self.Submerged()

Surface = Stop
