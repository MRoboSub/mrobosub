from umrsm import State
from periodic_io import PIO
import rospy
from typing import NamedTuple, Optional, List, Union
from abc import abstractmethod


class TimedState(State):
    """ base class for States which can be timed out. 

        expects an outcome called TimedOut and parameter named timeout.
        override handle_once_timedout iff cleanup is needed after timeout
    """

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        self.start_time = rospy.get_time()

    def handle(self) -> Optional[NamedTuple]:
        if rospy.get_time() - self.start_time >= self.timeout:
            return self.handle_once_timedout()
        return self.handle_if_not_timedout()

    @abstractmethod
    def handle_if_not_timedout(self) -> Optional[NamedTuple]:
        pass

    @abstractmethod
    def handle_once_timedout(self) -> NamedTuple:
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

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        self.start_time = rospy.get_time()
        self.waiting = False

    def handle(self) -> Optional[NamedTuple]:
        if not self.waiting:
            PIO.set_target_twist_surge(self.surge_speed)
            PIO.set_target_pose_heave(self.target_heave)

            if rospy.get_time() - self.start_time >= self.target_surge_time:
                PIO.set_target_twist_surge(0)
                self.waiting = True
                self.start_time = rospy.get_time()
        else:
            PIO.set_target_twist_surge(0)

            if rospy.get_time() - self.start_time >= self.wait_time:
                return self.handle_reached()

        return self.handle_unreached()

    @abstractmethod
    def handle_reached(self) -> Optional[NamedTuple]:
        pass

    @abstractmethod
    def handle_unreached(self) -> Optional[NamedTuple]:
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

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        self.start_time = rospy.get_time()
        self.timed_out_first = False

    def handle(self) -> Optional[NamedTuple]:
        if not self.timed_out_first:
            outcome = self.handle_first_phase()
            if rospy.get_time() - self.start_time >= self.phase_one_time:
                self.timed_out_first = True
                self.start_time = rospy.get_time()
        else:
            outcome = self.handle_second_phase()
            if rospy.get_time() - self.start_time >= self.phase_two_time:
                outcome = self.handle_once_timedout()

        return outcome

    @abstractmethod
    def handle_first_phase(self) -> Optional[NamedTuple]:
        pass

    @abstractmethod
    def handle_second_phase(self) -> Optional[NamedTuple]:
        pass

    @abstractmethod
    def handle_once_timedout(self) -> Optional[NamedTuple]:
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

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        self.timer = rospy.get_time()

    def handle_if_not_timedout(self) -> Optional[NamedTuple]:
        PIO.set_target_pose_yaw(self.target_yaw)

        if not PIO.is_yaw_within_threshold(self.yaw_threshold):
            self.timer = rospy.get_time()

        if rospy.get_time() - self.timer >= self.settle_time:
            return self.handle_reached()

        return self.handle_unreached()

    def handle_unreached(self) -> Optional[NamedTuple]:
        return None

    @abstractmethod
    def handle_reached(self) -> Optional[NamedTuple]:
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
    def handle_no_measurements(self) -> NamedTuple:
        pass
    
    @abstractmethod
    def handle_aligned(self) -> NamedTuple:
        pass
        
    def __init__(self, prev_outcome: NamedTuple) -> None:
        super().__init__(prev_outcome)
        PIO.activate_bot_cam()
        self.last_known_angle: Optional[float] = None
        self.iter = 0
        self.measurements: List[float] = []
                
    def handle_if_not_timedout(self) -> Union[NamedTuple, None]:
        PIO.set_target_twist_surge(0)
        self.iter += 1
        if self.iter < 50:
            return None
        if self.iter < 100:
            pm_resp = PIO.query_pathmarker()
            print(f'{pm_resp=}')
            if pm_resp is not None:
                self.measurements.append(pm_resp)
            return None
        if self.iter == 100:
            print('Calculating target')
            if len(self.measurements) < 20:
                return self.handle_no_measurements()
            self.target_angle = sum(self.measurements) / len(self.measurements)
            print(f'{self.target_angle=}')
            self.yaw_threshold_count = 0
        if self.iter >= 100:
            PIO.set_target_pose_yaw(self.target_angle)
            if PIO.is_yaw_within_threshold(self.yaw_threshold):
                self.yaw_threshold_count += 1
            else:
                self.yaw_threshold_count = 0
            if self.yaw_threshold_count > 30:
                return self.handle_aligned()
        return None

