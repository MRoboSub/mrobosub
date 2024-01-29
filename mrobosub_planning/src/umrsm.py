"""Contains the state machine implementation. You probably shouldn't read this unless you want to deal with wierd
Python metaprogramming."""

from __future__ import annotations
from abc import abstractmethod
from typing import (
    Dict,
    Mapping,
    Type,
    Tuple,
)
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from periodic_io import PIO
from typing import NamedTuple

STATE_TOPIC = "captain/current_state"
SOFT_STOP_SERVICE = "captain/soft_stop"

__all__ = (
    "Outcome",
    "TransitionMap",
    "State",
    "TimedState",
    "ForwardAndWait",
    "TurnToYaw",
    "StateMachine",
)


Outcome = NamedTuple


class InitTransition(Outcome):
    pass


class State:
    """States contain logic that will be executed by the StateMachine.

        Each State also contains class variables for each parameter on the parameter server, which
            can be accessed using self. Data that should be shared between calls of handle should be 
            set as an instance variable.
    """

    def __init__(self, prev_outcome: Outcome):
        self.prev_outcome = prev_outcome

    @abstractmethod
    def handle(self) -> Outcome:
        """Contains the logic to be run for a particular state.

        Is called repeatedly for each iteration of the state, including the first one.
        """
        pass


class TimedState(State):
    """ base class for States which can be timed out. 

        expects an outcome called TimedOut and parameter named timeout.
        override handle_once_timedout iff cleanup is needed after timeout
    """

    def __init__(self, prev_outcome: Outcome):
        super().__init__(prev_outcome)
        self.start_time = rospy.get_time()

    def handle(self) -> Outcome:
        if rospy.get_time() - self.start_time >= self.timeout:
            return self.handle_once_timedout()
        return self.handle_if_not_timedout()

    @abstractmethod
    def handle_if_not_timedout(self) -> Outcome:
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

    def __init__(self, prev_outcome: Outcome):
        super().__init__(prev_outcome)
        self.start_time = rospy.get_time()
        self.waiting = False

    def handle(self) -> Outcome:
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
    def handle_reached(self) -> Outcome:
        pass

    @abstractmethod
    def handle_unreached(self) -> Outcome:
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

    def __init__(self, prev_outcome: Outcome):
        super().__init__(prev_outcome)
        self.start_time = rospy.get_time()
        self.timed_out_first = False

    def handle(self) -> Outcome:
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
    def handle_first_phase(self) -> Outcome:
        pass

    @abstractmethod
    def handle_second_phase(self) -> Outcome:
        pass

    @abstractmethod
    def handle_once_timedout(self) -> Outcome:
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
    Unreached
    Reached
    TimedOut

    Must specify following parameters:
    target_yaw: float
    yaw_threshold: float
    settle_time: float
    timeout: float
    """

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)

        if not PIO.is_yaw_within_threshold(self.yaw_threshold):
            self.timer = rospy.get_time()

        if rospy.get_time() - self.timer >= self.settle_time:
            return self.handle_reached()

        return self.handle_unreached()

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

    @abstractmethod
    def handle_reached(self) -> Outcome:
        pass

    @abstractmethod
    def handle_unreached(self) -> Outcome:
        pass


class StateMachine:
    """ The main interface for running a system. """

    def __init__(
        self,
        name: str,
        transitions: Mapping[Type[Outcome], Type[State]],
        StartState: Type[State],
        StopState: Type[State],
    ):
        """ Creates a new state machine.

            You should call this code once for any particular run. 
            name: a string representing the name of the machine (used for overriding default parameters)
            transitions: a dictionary from Outcomes to States. Each of these should be the class itself, rather
                    than an instance of the class
            StartState: the class of the state to begin with
            StopState: the class of the state to end with. when this state is reach, its handle will be 
                called once, then the run method will return.
        """
        self.name = name
        self.StartState = StartState
        self.transitions = transitions
        self.StopState = StopState

        self._soft_stop_srv = rospy.Service(SOFT_STOP_SERVICE, Trigger, self.soft_stop)
        self.stop_signal_recvd = False

        # self._load_params(State, "globals", "defaults")
        # self._load_params(State, "globals", self.name)
        # states = set(transitions.values()).union({self.StartState})
        # for state in states:
        #     self._load_params(state, state.__module__, "mod", "defaults")
        #     self._load_params(state, state.__module__, "mod", self.name)
        #     self._load_params(
        #         state, state.__module__, state.__name__.lower(), "defaults"
        #     )
        #     self._load_params(
        #         state, state.__module__, state.__name__.lower(), self.name
        #     )

    # @staticmethod
    # def _load_params(
    #     state: Type[State], module: str, state_name, machine_name: str = ""
    # ) -> None:
    #     """Set all parameters in the module/state_name/machine_name namespace of the ROS parameter services as class
    #     variables in state."""
    #     if module == "__main__":
    #         module = "globals"
    #     namespace = (
    #         f"~{module}/{state_name}/{machine_name}"
    #         if machine_name
    #         else f"~{module}/{state_name}"
    #     )
    #     print(f"loading namespace {namespace}")
    #     for key, value in rospy.get_param(namespace, {}).items():
    #         print(f"\t{key}: {value}")
    #         setattr(state, key, value)  # read-only constants

    def soft_stop(self, data: TriggerRequest) -> Tuple[bool, str]:
        self.stop_signal_recvd = True
        return True, type(self.current_state).__qualname__

    def run(self) -> Outcome:
        """ Performs a run, beginning with the StartState and ending when it reaches StopState.
            
            Returns the Outcome from calling handle() on StopState.
        """
        rate = rospy.Rate(50)
        publisher = rospy.Publisher(STATE_TOPIC, String, queue_size=1)
        self.current_state = self.StartState(InitTransition())
        while type(self.current_state) != self.StopState:
            self.run_once(publisher)
            rate.sleep()
        publisher.publish(type(self.current_state).__qualname__)
        return self.current_state.handle()

    def run_once(self, state_topic_pub: rospy.Publisher):
        """ Runs one iteration of the state machine
        """
        state_topic_pub.publish(type(self.current_state).__qualname__)
        outcome = self.current_state.handle()
        outcome_type = type(outcome) if isinstance(outcome, Outcome) else outcome
        outcome_name = outcome_type.__qualname__
        if self.stop_signal_recvd:
            NextState = self.StopState
            outcome_name = "!! Abort !!"
        else:
            NextState = self.transitions[outcome_type]
        rospy.logdebug(
            f"{type(self.current_state).__qualname__} -> {NextState.__qualname__}"
        )
        if type(self.current_state) != NextState:
            rospy.loginfo(
                f"transition {type(self.current_state).__qualname__} --[{outcome_name}]--> {NextState.__qualname__}"
            )
            self.current_state = NextState(outcome)  # handle stop state


TransitionMap = Dict[Type[Outcome], Type[State]]
