from mrobosub_planning.mrobosub_planning.periodic_io import SegImageTarget
from mrobosub_planning.umrsm import State, Outcome
import rclpy
import rclpy.node
from typing import Optional, List, Union
from abc import abstractmethod


class TimedState(State):
    """base class for States which can be timed out.

    expects an outcome called TimedOut and parameter named timeout.
    override handle_once_timedout iff cleanup is needed after timeout
    """

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node):
        super().__init__(prev_outcome, node)
        self.start_time = self.io_node.get_clock().now().nanoseconds/(1e9)

    def handle(self) -> Optional[Outcome]:
        if self.io_node.get_clock().now().nanoseconds/(1e9) - self.start_time >= self.timeout:
            return self.handle_once_timedout()
        return self.handle_if_not_timedout()

    @abstractmethod
    def handle_if_not_timedout(self) -> Optional[Outcome]:
        pass

    @abstractmethod
    def handle_once_timedout(self) -> Outcome:
        pass

    @property
    @abstractmethod
    def timeout(self) -> float:
        pass


class ForwardAndWait(State):
    """
    Must specify the following outcomes:
    Unreached
    Reached

    Must specify the following parameters:
    target_heave: float
    target_surge_time: float
    wait_time: float
    surge_speed: float
    """

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node):
        super().__init__(prev_outcome, node)
        self.start_time = self.io_node.get_clock().now().nanoseconds/(1e9)
        self.waiting = False

    def handle(self) -> Optional[Outcome]:
        if not self.waiting:
            self.io_node.set_target_twist_surge(self.surge_speed)
            self.io_node.set_target_pose_heave(self.target_heave)

            if self.io_node.get_clock().now().nanoseconds/(1e9) - self.start_time >= self.target_surge_time:
                self.io_node.set_target_twist_surge(0)
                self.waiting = True
                self.start_time = self.io_node.get_clock().now().nanoseconds/(1e9)
        else:
            self.io_node.set_target_twist_surge(0)

            if self.io_node.get_clock().now().nanoseconds/(1e9) - self.start_time >= self.wait_time:
                return self.handle_reached()

        return self.handle_unreached()

    @abstractmethod
    def handle_reached(self) -> Optional[Outcome]:
        pass

    @abstractmethod
    def handle_unreached(self) -> Optional[Outcome]:
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


class DoubleTimedState(State):
    """
    Must specify the following outcomes:
    Unreached
    Reached

    Must specify the following parameters:
    phase_one_time: float
    phase_two_time: float
    """

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node):
        super().__init__(prev_outcome, node)
        self.start_time = self.io_node.get_clock().now().nanoseconds/(1e9)
        self.timed_out_first = False

    def handle(self) -> Optional[Outcome]:
        if not self.timed_out_first:
            outcome = self.handle_first_phase()
            if self.io_node.get_clock().now().nanoseconds/(1e9) - self.start_time >= self.phase_one_time:
                self.timed_out_first = True
                self.start_time = self.io_node.get_clock().now().nanoseconds/(1e9)
        else:
            outcome = self.handle_second_phase()
            if self.io_node.get_clock().now().nanoseconds/(1e9) - self.start_time >= self.phase_two_time:
                outcome = self.handle_once_timedout()

        return outcome

    @abstractmethod
    def handle_first_phase(self) -> Optional[Outcome]:
        pass

    @abstractmethod
    def handle_second_phase(self) -> Optional[Outcome]:
        pass

    @abstractmethod
    def handle_once_timedout(self) -> Optional[Outcome]:
        pass

    @property
    @abstractmethod
    def phase_one_time(self) -> float:
        pass

    @property
    @abstractmethod
    def phase_two_time(self) -> float:
        pass


class TurnToYaw(TimedState):
    """
    Must specify following outcomes:
    Reached
    TimedOut

    Must specify following parameters:
    target_yaw: float
    yaw_threshold: float
    settle_time: float
    timeout: float
    """

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node):
        super().__init__(prev_outcome, node)
        self.timer = self.io_node.get_clock().now().nanoseconds/(1e9)

    def handle_if_not_timedout(self) -> Optional[Outcome]:
        self.io_node.set_target_pose_yaw(self.target_yaw)

        if not self.io_node.is_yaw_within_threshold(self.yaw_threshold):
            self.timer = self.io_node.get_clock().now().nanoseconds/(1e9)

        if self.io_node.get_clock().now().nanoseconds/(1e9) - self.timer >= self.settle_time:
            return self.handle_reached()

        return self.handle_unreached()

    def handle_unreached(self) -> Optional[Outcome]:
        return None

    @abstractmethod
    def handle_reached(self) -> Optional[Outcome]:
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

    @property
    @abstractmethod
    def timeout(self) -> float:
        pass


class AlignPathmarker(TimedState):
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

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node) -> None:
        super().__init__(prev_outcome, node)
        self.io_node.activate_bot_cam()
        self.last_known_angle: Optional[float] = None
        self.iter = 0
        self.measurements: List[float] = []

    def handle_if_not_timedout(self) -> Union[Outcome, None]:
        self.io_node.set_target_twist_surge(0)
        self.iter += 1
        if self.iter < 50:
            return None
        if self.iter < 100:
            pm_resp = self.io_node.query_seg_image(SegImageTarget.PATHMARKER)
            self.io_node.get_logger().info({f"{pm_resp=}"})
            if pm_resp is not None and pm_resp.found and pm_resp.valid:
                self.measurements.append(pm_resp.direction)
            return None
        if self.iter == 100:
            self.io_node.get_logger().info({"Calculating target"})
            if len(self.measurements) < 20:
                self.io_node.deactivate_bot_cam()
                return self.handle_no_measurements()
            self.target_angle = sum(self.measurements) / len(self.measurements)
            self.io_node.get_logger().info({f"{self.target_angle=}"})
            self.yaw_threshold_count = 0
        if self.iter >= 100:
            self.io_node.set_target_pose_yaw((- self.target_angle) % 360)
            if self.io_node.is_yaw_within_threshold(self.yaw_threshold):
                self.yaw_threshold_count += 1
            else:
                self.yaw_threshold_count = 0
            if self.yaw_threshold_count > 30:
                self.io_node.deactivate_bot_cam()
                return self.handle_aligned()
        return None



class CenterOnPathmarker(TimedState):
    @abstractmethod
    def handle_centered(self) -> Outcome:
        pass

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node):
        super().__init__(prev_outcome, node)
        self.io_node.activate_bot_cam()
        self.centered_count = 0

    def handle_if_not_timedout(self) -> Optional[Outcome]:
        pm_resp = self.io_node.query_seg_image(SegImageTarget.PATHMARKER)
        self.io_node.set_target_pose_yaw(0)
        if pm_resp is None or not pm_resp.found or not pm_resp.valid:
            self.io_node.set_target_twist_surge(0.0)
            self.io_node.set_target_twist_sway(0.0)
            return None

        x_diff = pm_resp.x_position - 0.5 #0.4
        y_diff = pm_resp.y_position - 0.5 #-0.4
        self.io_node.set_target_twist_sway(3 * x_diff)
        self.io_node.set_target_twist_surge(-3 * y_diff)
        if abs(x_diff) < 0.1 and abs(y_diff) < 0.1:
            self.centered_count += 1
        else:
            self.centered_count = 0
        if self.centered_count > 50:
            return self.handle_centered()
        return None
