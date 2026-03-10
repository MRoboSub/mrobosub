from mrobosub_planning.abstract_states import ForwardAndWait, TimedState
from mrobosub_planning.umrsm import Outcome


class Forward5Seconds(ForwardAndWait):
    class Reached(Outcome):
        pass

    class Unreached(Outcome):
        pass

    target_heave: float      = 0.5
    target_surge_time: float = 5.
    wait_time: float         = 5.
    surge_speed: float       = 0.1

    def handle_reached(self) -> Outcome:
        return self.Reached()
    
    def handle_unreached(self) -> Outcome:
        return self.Unreached()
    


class ComeToSurface(TimedState):
    class Surfaced(Outcome):
        pass

    class Failed(Outcome):
        pass

    target_heave: float = 0.05
    heave_threshold: float = 0.1
    timeout: float = 15.0
    yaw_threshold: float = 2.
    target_yaw: float = 0.

    def handle_if_not_timedout(self) -> Surfaced | Failed:
        self.io_node.set_target_pose_heave(self.target_heave)
        self.io_node.set_target_pose_yaw(self.target_yaw)

        if self.io_node.is_heave_within_threshold(
            self.heave_threshold
        ) and self.io_node.is_yaw_within_threshold(self.yaw_threshold):
            return self.Surfaced()
        return self.Failed()
    
    def handle_once_timedout(self) -> Failed:
        return self.Failed()
        

    