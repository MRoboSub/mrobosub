from mrobosub_planning.umrsm import State, Outcome
from mrobosub_planning.periodic_io import Interface
from mrobosub_planning.abstract_states import TimedState


class Start(State):
    class Complete(Outcome):
        pass

    def handle(self) -> Complete:
        return self.Complete()


class Submerge(TimedState):
    class Submerged(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    target_heave: float = 0.75
    heave_threshold: float = 0.1
    timeout: float = 15.0
    yaw_threshold: float = 2.0
    target_yaw: float = 0.0

    def handle_if_not_timedout(self) -> Submerged | None:
        self.io.set_target_pose_heave(self.target_heave)
        self.io.set_target_pose_yaw(self.target_yaw)

        if self.io.is_heave_within_threshold(
            self.heave_threshold
        ) and self.io.is_yaw_within_threshold(self.yaw_threshold):
            return self.Submerged()
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class Stop(State):
    class Surfaced(Outcome):
        pass

    def __init__(self, prev_outcome: Outcome, io: Interface):
        super().__init__(prev_outcome, io)
        self.io.reset_target_twist()
        self.rate = self.io.node.create_rate(50)

    def handle(self) -> None:
        for _ in range(20):
            self.io.reset_target_twist()
            self.rate.sleep()
        return None


Surface = Stop
