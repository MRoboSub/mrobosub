from mrobosub_planning.umrsm import State, Outcome
from mrobosub_planning.periodic_io import Interface
import rclpy
import rclpy.node
from abc import abstractmethod


class TimedState(State):
    """Base class for States which can be timed out.

    Must implement the following functions:
        handle_if_node_timedout
        handle_once_timedout

    Must specify the following parameters:
        timeout
    """

    def __init__(self, prev_outcome: Outcome, io: Interface):
        super().__init__(prev_outcome, io)
        self.clock = self.io.node.get_clock()
        self.start_time = self.time()

    def handle(self) -> Outcome | None:
        if self.state_runtime() >= self.timeout:
            return self.handle_once_timedout()
        return self.handle_if_not_timedout()

    def time(self) -> float:
        return self.clock.now().nanoseconds / 1e9

    def state_runtime(self) -> float:
        return self.time() - self.start_time

    @abstractmethod
    def handle_if_not_timedout(self) -> Outcome | None:
        pass

    @abstractmethod
    def handle_once_timedout(self) -> Outcome:
        pass

    @property
    @abstractmethod
    def timeout(self) -> float:
        pass


class DoubleTimedState(TimedState):
    """
    Must implement the following functions:
        handle_first_phase
        handle_second_phase
        handle_once_timedout

    Must specify the following parameters:
        phase_one_time: float
        phase_two_time: float
    """

    def __init__(self, prev_outcome: Outcome, io: Interface):
        super().__init__(prev_outcome, io)

    def handle(self) -> Outcome | None:
        if self.state_runtime() < self.phase_one_time:
            return self.handle_first_phase()
        else:
            return self.handle_second_phase()

    @property
    def timeout(self) -> float:
        return self.phase_one_time + self.phase_two_time

    @abstractmethod
    def handle_first_phase(self) -> Outcome | None:
        pass

    @abstractmethod
    def handle_second_phase(self) -> Outcome | None:
        pass

    @property
    @abstractmethod
    def phase_one_time(self) -> float:
        pass

    @property
    @abstractmethod
    def phase_two_time(self) -> float:
        pass


class ForwardAndWait(DoubleTimedState):
    """
    Must implement the following functions:
        handle_reached

    May optionally override the following functions:
        handle_unreached

    Must specify the following parameters:
        target_heave: float
        target_surge_time: float
        wait_time: float
        surge_speed: float
    """

    def __init__(self, prev_outcome: Outcome, io: Interface):
        super().__init__(prev_outcome, io)

    def handle_first_phase(self) -> Outcome | None:
        self.io.set_target_twist_surge(self.surge_speed)
        self.io.set_target_pose_heave(self.target_heave)
        return self.handle_unreached()

    def handle_second_phase(self) -> Outcome | None:
        self.io.set_target_twist_surge(0.0)
        return self.handle_unreached()

    def handle_once_timedout(self) -> Outcome:
        return self.handle_reached()

    @property
    def phase_one_time(self) -> float:
        return self.target_surge_time

    @property
    def phase_two_time(self) -> float:
        return self.wait_time

    def handle_unreached(self) -> Outcome | None:
        return None

    @abstractmethod
    def handle_reached(self) -> Outcome:
        pass

    @property
    @abstractmethod
    def target_heave(self) -> float:
        pass

    @property
    @abstractmethod
    def target_surge_time(self) -> float:
        pass

    @property
    @abstractmethod
    def wait_time(self) -> float:
        pass

    @property
    @abstractmethod
    def surge_speed(self) -> float:
        pass


class TurnToYaw(TimedState):
    """
    Must implement the following functions:
        handle_reached

    May optionally override the following functions:
        handle_unreached

    Must specify the following parameters:
        target_yaw: float
        yaw_threshold: float
        settle_time: float
        timeout: float
    """

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node):
        super().__init__(prev_outcome, node)
        self.settled_since = float("inf")

    def handle_if_not_timedout(self) -> Outcome | None:
        self.io.set_target_pose_yaw(self.target_yaw)

        if not self.io.is_yaw_within_threshold(self.yaw_threshold):
            self.settled_since = self.time()

        if self.time() - self.settled_since >= self.settle_time:
            return self.handle_reached()

        return self.handle_unreached()

    def handle_unreached(self) -> Outcome | None:
        return None

    @abstractmethod
    def handle_reached(self) -> Outcome | None:
        pass

    @property
    @abstractmethod
    def target_yaw(self) -> float:
        pass

    @property
    @abstractmethod
    def yaw_threshold(self) -> float:
        pass

    @property
    @abstractmethod
    def settle_time(self) -> float:
        pass


class AlignPathmarker(TimedState):
    """
    Must implement the following functions:
        handle_no_measurements
        handle_aligned
        handle_once_timedout

    Must specify the following parameters:
        yaw_threshold: float
        timeout: float
    """

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node) -> None:
        super().__init__(prev_outcome, node)
        self.io.activate_bot_cam()
        self.last_known_angle: float | None = None
        self.iter = 0
        self.measurements: list[float] = []

    def handle_if_not_timedout(self) -> Outcome | None:
        self.io.set_target_twist_surge(0)
        self.iter += 1
        if self.iter < 50:
            return None
        if self.iter < 100:
            pm_resp = self.io.query_pathmarker()  # type: ignore # TODO: remove once ML is migrated
            self.io.logger.info({f"{pm_resp=}"})
            if pm_resp is not None:
                self.measurements.append(pm_resp)
            return None
        if self.iter == 100:
            self.io.logger.info({"Calculating target"})
            if len(self.measurements) < 20:
                return self.handle_no_measurements()
            self.target_angle = sum(self.measurements) / len(self.measurements)
            self.io.logger.info({f"{self.target_angle=}"})
            self.yaw_threshold_count = 0
        if self.iter >= 100:
            self.io.set_target_pose_yaw(self.target_angle)
            if self.io.is_yaw_within_threshold(self.yaw_threshold):
                self.yaw_threshold_count += 1
            else:
                self.yaw_threshold_count = 0
            if self.yaw_threshold_count > 30:
                return self.handle_aligned()
        return None

    @property
    @abstractmethod
    def yaw_threshold(self) -> float:
        pass

    @abstractmethod
    def handle_no_measurements(self) -> Outcome:
        pass

    @abstractmethod
    def handle_aligned(self) -> Outcome:
        pass


class CenterOnPathmarker(TimedState):
    """
    Must implement the following functions:
        handle_aligned
        handle_once_timedout

    Must specify the following parameters:
        timeout: float
    """

    def __init__(self, prev_outcome: Outcome, io: Interface):
        super().__init__(prev_outcome, io)
        self.io.activate_bot_cam()
        self.centered_count = 0

    def handle_if_not_timedout(self) -> Outcome | None:
        pm_resp = self.io.query_pathmarker_full()  # type: ignore # TODO: remove once ML is migrated
        if pm_resp is None:
            self.io.set_target_twist_surge(0.0)
            self.io.set_target_twist_sway(0.0)
            return None

        x_diff = pm_resp.centroid_x - 0.5
        y_diff = pm_resp.centroid_y - 0.5
        self.io.set_target_twist_sway(3 * x_diff)
        self.io.set_target_twist_surge(-3 * y_diff)
        if abs(x_diff) < 0.1 and abs(y_diff) < 0.1:
            self.centered_count += 1
        else:
            self.centered_count = 0
        if self.centered_count > 50:
            return self.handle_aligned()
        return None

    @abstractmethod
    def handle_aligned(self) -> Outcome:
        pass
