from umrsm import *
from common_states import *
import prequal_front
from abstract_states import TurnToYaw, ForwardAndWait


class TurnAroundMarker(TurnToYaw):
    class Reached(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    target_yaw: float = 90
    yaw_threshold: float = 2
    timeout: float = 10
    settle_time: float = 1

    def handle_reached(self) -> Reached:
        return self.Reached()

    def handle_unreached(self) -> None:
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class MovePastMarker(ForwardAndWait):
    class Reached(Outcome):
        pass

    target_heave: float = 1.4
    target_surge_time: float = 5
    surge_speed: float = 1.5
    wait_time: float = 1

    def handle_reached(self) -> Reached:
        return self.Reached()

    def handle_unreached(self) -> None:
        return None


class TurnToGate(TurnToYaw):
    class Reached(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    target_yaw: float = -175
    yaw_threshold: float = 2
    timeout: float = 10
    settle_time: float = 1

    def handle_reached(self) -> Reached:
        return self.Reached()

    def handle_unreached(self) -> None:
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class LeaveMarker(ForwardAndWait):
    class Reached(Outcome):
        pass

    target_heave: float = 1.4
    target_surge_time: float = 10
    surge_speed: float = 1.5
    wait_time: float = 1

    def handle_reached(self) -> Reached:
        return self.Reached()

    def handle_unreached(self) -> None:
        return None


class ReturnToGate(ForwardAndWait):
    class Reached(Outcome):
        pass

    target_heave: float = 1.4
    target_surge_time: float = 224
    surge_speed: float = 1.5
    wait_time: float = 1

    def handle_reached(self) -> Outcome:
        return self.Reached()

    def handle_unreached(self) -> None:
        return None


transitions: TransitionMap = prequal_front.transitions.copy()
transitions.update(
    {
        prequal_front.ApproachMarker.Reached: TurnAroundMarker,

        TurnAroundMarker.Reached: MovePastMarker,
        TurnAroundMarker.TimedOut: MovePastMarker,

        MovePastMarker.Reached: TurnToGate,

        TurnToGate.Reached: LeaveMarker,
        TurnToGate.TimedOut: LeaveMarker,

        LeaveMarker.Reached: ReturnToGate,

        ReturnToGate.Reached: Stop,
    }
)
