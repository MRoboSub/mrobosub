from typing import Optional, Union
from mrobosub_planning.umrsm import State, Outcome
from mrobosub_planning.abstract_states import TimedState
from rclpy.node import Node as RosNode


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

    target_heave: float = 15
    heave_threshold: float = 0.1
    timeout: float = 10
    yaw_threshold: float = 2.
    target_yaw: float = 0.

    def handle_if_not_timedout(self) -> Union[Submerged, None]:
        self.io_node.set_target_pose_heave(self.target_heave)
        # self.io_node.set_target_twist_yaw(0)

        if self.io_node.is_heave_within_threshold(
            self.heave_threshold
        ) and self.io_node.is_yaw_within_threshold(self.yaw_threshold):
            return self.Submerged()
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()
    

class Forward10(TimedState):
    class ReachedGate(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    target_heave: float = 0.5
    heave_threshold: float = 0.1
    timeout: float = 10
    yaw_threshold: float = 2.
    target_yaw: float = 0.

    def handle_if_not_timedout(self) -> Union[ReachedGate, None]:
        # self.io_node.set_target_twist_heave(0.4)
        self.io_node.set_target_twist_surge(1.5)
        self.io_node.set_target_pose_yaw(0)
        self.io_node.set_target_pose_heave(15)
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()
    

class Turn180_2(TimedState):
    class ReachedGate(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    target_heave: float = 0.5
    heave_threshold: float = 0.1
    timeout: float = 10
    yaw_threshold: float = 2.
    target_yaw: float = 0.

    def handle_if_not_timedout(self) -> Union[ReachedGate, None]:
        self.io_node.set_target_pose_heave(15)
        self.io_node.set_target_twist_surge(0)
        self.io_node.set_target_pose_yaw(10)
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class ReturnHome(TimedState):
    class ReachedGate(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    target_heave: float = 0.5
    heave_threshold: float = 0.1
    timeout: float = 15
    yaw_threshold: float = 2.
    target_yaw: float = 0.

    def handle_if_not_timedout(self) -> Union[ReachedGate, None]:
        self.io_node.set_target_twist_surge(2)
        self.io_node.set_target_pose_yaw(10)
        self.io_node.set_target_pose_heave(15)
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()
    

class ComeUp(TimedState):
    class ReachedGate(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    target_heave: float = 0.5
    heave_threshold: float = 0.1
    timeout: float = 5
    yaw_threshold: float = 2.
    target_yaw: float = 0.

    def handle_if_not_timedout(self) -> Union[ReachedGate, None]:
        self.io_node.set_target_pose_heave(0)
        self.io_node.set_target_twist_surge(0)
        self.io_node.set_target_pose_yaw(0)
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class RevertCoinFlip(TimedState):
    class Reverted(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    timeout:float = 15.0

    def handle_if_not_timedout(self) -> Union[Reverted, None]:
        self.io_node.set_target_pose_yaw(0)
        self.io_node.set_target_pose_heave(15)
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()



class TimeoutFor30(TimedState):
    class ReachedGate(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    target_heave: float = 0.5
    heave_threshold: float = 0.1
    timeout: float = 30
    yaw_threshold: float = 2.
    target_yaw: float = 0.

    def handle_if_not_timedout(self) -> Union[ReachedGate, None]:
        self.io_node.set_target_twist_heave(0)
        self.io_node.set_target_twist_surge(0)
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()

class Stop(State):
    class Surfaced(Outcome):
        pass

    def __init__(self, prev_outcome: Outcome, node: RosNode):
        super().__init__(prev_outcome, node)
        self.io_node.reset_target_twist()
        self.rate = self.io_node.create_rate(50)

    def handle(self) -> None:
        for _ in range(20):
            self.io_node.reset_target_twist()
            self.rate.sleep()
        return None


Surface = Stop
