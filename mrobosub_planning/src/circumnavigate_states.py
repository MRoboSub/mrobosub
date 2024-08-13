from abstract_states import TimedState, TurnToYaw
from periodic_io import PIO
import rospy
from typing import NamedTuple, Optional, Type, Union


class CircumnavigateOpenContinuous(TimedState):
    class Finished(NamedTuple):
        pass

    timeout: float = 10
    yaw_twist: float = 0.2
    surge_twist: float = 0.2

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        if getattr(prev_outcome, "ccw", False):
            self.dir = -1
        else:
            self.dir = 1

    def handle_if_not_timedout(self) -> None:
        PIO.set_target_twist_yaw(self.yaw_twist * self.dir)
        PIO.set_target_twist_surge(self.surge_twist * self.dir)
        return None

    def handle_once_timedout(self) -> Finished:
        return self.Finished()


class CircumnavigateOpenDiscreteData(NamedTuple):
    cum_angle: float
    curr_angle: float
    ccw: bool


class CircumnavigateOpenDiscreteDiamondTurns(TurnToYaw):
    class FinishedStep(CircumnavigateOpenDiscreteData):
        pass

    class Complete(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    timeout: float = 8.0
    yaw_threshold: float = 7.5
    settle_time: float = 1.0
    angle_per_iter: float = 360.0 / 4.0 + 3 # add 3 degrees each turn to account for drift
    initial_turn: float = angle_per_iter / 2.0

    @property
    def target_yaw(self) -> float:
        return self._target_yaw

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        self.dir = 1 # always go clockwise for matching points

        if not isinstance(prev_outcome, CircumnavigateOpenDiscreteData):
            self.cum_angle = 0.0
            self._target_yaw = (self.cum_angle - self.initial_turn * self.dir) % 360
        else:
            self.cum_angle = prev_outcome.cum_angle + abs(self.angle_per_iter)
            self._target_yaw = (prev_outcome.curr_angle + self.angle_per_iter * self.dir) % 360
        PIO.set_target_twist_surge(0.0)

    def handle_unreached(self) -> None:
        PIO.set_target_twist_surge(0.0)
        return None

    def handle_reached(self) -> Optional[Union[FinishedStep, Complete]]:
        PIO.set_target_twist_surge(0.0)
        if self.cum_angle >= 350.0:
            return self.Complete()
        return self.FinishedStep(self.cum_angle, self.target_yaw, self.dir == -1)

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0.0)
        return self.TimedOut()


class CircumnavigateOpenDiscreteMove(TimedState):
    class FinishedStep(CircumnavigateOpenDiscreteData):
        pass

    timeout: float = 11.0
    surge_twist: float = 0.15

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        if not isinstance(prev_outcome, CircumnavigateOpenDiscreteData):
            raise TypeError(f"Expected type CircumnavigateOpenDiscreteData, received {prev_outcome}")
        self.cum_angle = prev_outcome.cum_angle
        self.curr_yaw = prev_outcome.curr_angle
        self.ccw = prev_outcome.ccw

    @classmethod
    def is_valid_income_type(cls, outcome_type: Type[NamedTuple]) -> bool:
        return issubclass(outcome_type, CircumnavigateOpenDiscreteData)

    def handle_if_not_timedout(self) -> None:
        PIO.set_target_pose_yaw(self.curr_yaw)
        PIO.set_target_twist_surge(self.surge_twist)

    def handle_once_timedout(self) -> FinishedStep:
        return self.FinishedStep(self.cum_angle, self.curr_yaw, self.ccw)
