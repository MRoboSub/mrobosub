from typing import Union
from umrsm import State, NamedTuple
from abstract_states import TimedState
from periodic_io import PIO
import rospy

class Start(State):
    class Complete(NamedTuple): pass
    
    def handle(self):
        return self.Complete()

class Submerge(TimedState):
    class Submerged(NamedTuple): pass
    class TimedOut(NamedTuple): pass
    
    target_heave: float = 0.25
    heave_threshold: float = 0.1
    timeout: float = 15
    yaw_threshold: float = 2
    target_yaw: float = 0
    
    def handle_if_not_timedout(self) -> Union[Submerged, None]:
        PIO.set_target_pose_heave(self.target_heave)
        PIO.set_target_pose_yaw(self.target_yaw)

        if (PIO.is_heave_within_threshold(self.heave_threshold) and 
                PIO.is_yaw_within_threshold(self.yaw_threshold)):
            return self.Submerged()
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()

class Stop(State):
    class Surfaced(NamedTuple): pass

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        PIO.reset_target_twist()
        self.rate = rospy.Rate(50)
    
    def handle(self) -> None:
        for _ in range(20):
            PIO.reset_target_twist()
            self.rate.sleep()
        return None

Surface = Stop
