from typing import Optional, Union
from umrsm import State, NamedTuple
from abstract_states import ForwardAndWait, TimedState, TurnToYaw
from periodic_io import PIO
import rospy


class Start(State):
    class Complete(NamedTuple):
        pass

    def handle(self) -> Complete:
        return self.Complete()


class Submerge(TimedState):
    class Submerged(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    target_heave: float = 0.75
    heave_threshold: float = 0.1
    timeout: float = 15
    yaw_threshold: float = 2
    target_yaw: float = 0

    def handle_if_not_timedout(self) -> Union[Submerged, None]:
        PIO.set_target_pose_heave(self.target_heave)
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_heave_within_threshold(
            self.heave_threshold
        ) and PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Submerged()
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class Stop(State):
    class Surfaced(NamedTuple):
        pass

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        PIO.reset_target_twist()
        self.rate = rospy.Rate(50)

    def handle(self) -> None:
        for _ in range(20):
            PIO.reset_target_twist()
            self.rate.sleep()
        return None

class Forward(ForwardAndWait):
    class Moved(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    # Here are some "defaults"
    # The assumption is here that you would use 
    # the state metaprogramming to set these.
    _target_heave:      float =  0.0
    _target_surge_time: float = 10.0
    _wait_time:         float = 10.0
    _surge_speed:       float = 10.0

    @property
    def target_heave(self) -> float:
        return self._target_heave

    @property
    def target_surge_time(self) -> float:
        return self._target_surge_time

    @property
    def wait_time(self) -> float:
        return self._wait_time

    @property
    def surge_speed(self) -> float:
        return self._surge_speed

    def handle_reached(self) -> Optional[NamedTuple]:
        return self.Moved()

    def handle_unreached(self) -> Optional[NamedTuple]:
        return self.TimedOut()


class Turn(TurnToYaw):
    class Complete(NamedTuple):
        pass

    _target_yaw:    float =  0.0
    _yaw_threshold: float =  2.0
    _settle_time:   float = 10.0
    _timeout:       float = 10.0

    @property
    def target_yaw(self) -> float:
        return self._target_yaw

    @property
    def yaw_threshold(self) -> float:
        return self._yaw_threshold

    @property
    def settle_time(self) -> float:
        return self._settle_time

    @property
    def timeout(self) -> float:
        return self._timeout
    
    def handle_reached(self) -> Optional[NamedTuple]:
        return self.Complete()


Surface = Stop
