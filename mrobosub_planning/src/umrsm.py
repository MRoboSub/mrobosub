"""Contains the state machine implementation. You probably shouldn't read this unless you want to deal with wierd
Python metaprogramming."""

from __future__ import annotations
from abc import abstractmethod
from typing import (
    Dict,
    Optional,
    Type,
    Tuple,
)
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from typing import NamedTuple

STATE_TOPIC = "captain/current_state"
SOFT_STOP_SERVICE = "captain/soft_stop"

__all__ = (
    "NamedTuple",
    "TransitionMap",
    "State",
    "StateMachine",
)


class InitTransition(NamedTuple):
    pass


class SoftStopTransition(NamedTuple):
    pass


class State:
    """States contain logic that will be executed by the StateMachine.

    Each State also contains class variables for each parameter on the parameter server, which
        can be accessed using self. Data that should be shared between calls of handle should be
        set as an instance variable.
    """

    def __init__(self, prev_outcome: NamedTuple):
        self.prev_outcome = prev_outcome

    @abstractmethod
    def handle(self) -> Optional[NamedTuple]:
        """Contains the logic to be run for a particular state.

        Is called repeatedly for each iteration of the state, including the first one.
        """
        pass

    @classmethod
    def is_valid_income_type(cls, outcome_type: Type[NamedTuple]) -> bool:
        return True


TransitionMap = Dict[Type[NamedTuple], Type[State]]


class StateMachine:
    """The main interface for running a system."""

    def __init__(
        self,
        name: str,
        transitions: TransitionMap,
        StartState: Type[State],
        StopState: Type[State],
    ):
        """Creates a new state machine.

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

    def soft_stop(self, data: TriggerRequest) -> Tuple[bool, str]:
        self.stop_signal_recvd = True
        return True, type(self.current_state).__qualname__

    def run(self) -> Optional[NamedTuple]:
        """Performs a run, beginning with the StartState and ending when it reaches StopState.

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

    def run_once(self, state_topic_pub: rospy.Publisher) -> None:
        """Runs one iteration of the state machine"""
        state_topic_pub.publish(type(self.current_state).__qualname__)

        outcome = self.current_state.handle()
        if self.stop_signal_recvd:
            if outcome is None:
                outcome = SoftStopTransition()
            NextState = self.StopState
            outcome_name = "!! Abort !!"
            rospy.loginfo(f"Aborting from state {type(self.current_state).__qualname__} and moving to stop state")
        else:
            if outcome is None:
                return
            outcome_type = type(outcome)
            outcome_name = outcome_type.__qualname__
            NextState = self.transitions[outcome_type]

            if type(self.current_state) == NextState:
                rospy.logdebug(f"{type(self.current_state).__qualname__} contains a type which returns itself!")

        rospy.loginfo(f"transition {type(self.current_state).__qualname__} --[{outcome_name}]--> {NextState.__qualname__}")
        self.current_state = NextState(outcome)
