from typing import Optional, Union
from umrsm import State, Outcome
from abstract_states import TimedState
from periodic_io import PIO
import rospy


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


# TODO: This class needs to be reworked
class MoveToXY(TimedState):
    class Reached(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    def __init__(self, prev_outcome: Outcome):
        super().__init__(prev_outcome)
        self.timer = rospy.get_time()

    def handle_if_not_timedout(self) -> Optional[Outcome]:
        PIO.set_target_pose_x(self.target_x)
        PIO.set_target_pose_y(self.target_y)

        desired_angle = PIO.calculate_yaw_to_target()
        PIO.set_target_pose_yaw(desired_angle)

        if not PIO.is_yaw_within_threshold(self.yaw_threshold):
            self.timer = rospy.get_time()

        if not self._reached_angle and rospy.get_time() - self.timer >= self.settle_time:
            self._reached_angle = True
            self.timer = rospy.get_time()

        if self._reached_angle:
            if not PIO.is_magnitude_within_threshold(self.magnitude_threshold):
                self.timer = rospy.get_time()
                error = self.kP * PIO.calculate_distance_to_target()
                heave = min(error, self.max_heave)
                PIO.set_target_twist_heave(heave)

            if rospy.get_time() - self.timer >= self.settle_time:
                PIO.set_target_twist_heave(0)
                return self.Reached()

        return None

    def handle_once_timedout(self) -> Outcome:
        return self.TimedOut()

    target_x: float = 0.0
    target_y: float = 0.0
    magnitude_threshold: float = 0.1
    yaw_threshold: float = 2.0
    settle_time: float = 1.0  # should we use a separate angle settle time and position settle time
    timeout: float = 20.0

    kP: float = 0.01  # this is the coefficient of the proportional term in PID
    max_heave: float = 0.3

    _reached_angle: bool = False


class Stop(State):
    class Surfaced(Outcome):
        pass

    def __init__(self, prev_outcome: Outcome):
        super().__init__(prev_outcome)
        PIO.reset_target_twist()
        self.rate = rospy.Rate(50)

    def handle(self) -> None:
        for _ in range(20):
            PIO.reset_target_twist()
            self.rate.sleep()
        return None


Surface = Stop
