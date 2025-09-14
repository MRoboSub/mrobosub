from typing import Optional, Union

from fpdf import Align
from mrobosub_planning.umrsm import State, Outcome
from mrobosub_planning.abstract_states import TimedState
import rclpy


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
    yaw_threshold: float = 2.
    target_yaw: float = 0.

    def handle_if_not_timedout(self) -> Union[Submerged, None]:
        self.io_node.set_target_pose_heave(self.target_heave)
        self.io_node.set_target_pose_yaw(self.target_yaw)

        if self.io_node.is_heave_within_threshold(
            self.heave_threshold
        ) and self.io_node.is_yaw_within_threshold(self.yaw_threshold):
            return self.Submerged()
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class AlignToYaw(TimedState):
    target_yaw = 0.0
    yaw_threshold = 2.0

    class Aligned(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    def handle_if_not_timedout(self) -> Union[Aligned, None]:
        self.io_node.set_target_pose_yaw(self.target_yaw)

        if self.io_node.is_yaw_within_threshold(self.yaw_threshold):
            return self.Aligned()
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class Stop(State):
    class Surfaced(Outcome):
        pass

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node):
        super().__init__(prev_outcome, node)
        self.io_node.reset_target_twist()
        self.rate = self.io_node.create_rate(50)

    def handle(self) -> None:
        for _ in range(20):
            self.io_node.reset_target_twist()
            self.rate.sleep()
        return None


Surface = Stop
