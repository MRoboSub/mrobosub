"""Contains the state machine implementation. You probably shouldn't read this unless you want to deal with wierd
Python metaprogramming."""

from __future__ import annotations
from abc import abstractmethod
from typing import (
    Any,
    Dict,
    Optional,
    Type,
    Tuple,
    TYPE_CHECKING,
)
import warnings
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from dataclasses import dataclass
from typing_extensions import dataclass_transform, Self
from mrobosub_planning.periodic_io import PIO

STATE_TOPIC = "captain/current_state"
SOFT_STOP_SERVICE = "captain/soft_stop"

__all__ = (
    "TransitionMap",
    "Outcome",
    "InitTransition",
    "SoftStopTransition",
    "State",
    "StateMachine",
)


@dataclass_transform()
class OutcomeMeta(type):
    _state: type[State]

    def __new__(cls, *args: Any, **kwargs: Any) -> "OutcomeMeta":
        """
        This removes the need to annotate all outcomes with @dataclass
        """
        return dataclass(super().__new__(cls, *args, **kwargs))

    def __repr__(self) -> str:
        if hasattr(self, "_state"):
            return f"<outcome {self.__name__} of state {self._state}>"
        return f"<outcome {self.__name__}>"


class Outcome(metaclass=OutcomeMeta):
    pass


class InitTransition(Outcome):
    pass


class SoftStopTransition(Outcome):
    pass


class StateMeta(type):
    _outcomes: dict[str, Type[Outcome]]

    def __new__(cls, name: str, bases: tuple, dict_: dict) -> "StateMeta":
        state = super().__new__(cls, name, bases, dict_)
        if TYPE_CHECKING:
            assert issubclass(state, State)
        for v in dict_.values():
            if isinstance(v, type) and issubclass(v, Outcome):
                v._state = state
        return state

    def _rendered_repr(self) -> str:
        if not hasattr(self, "_param_overrides"):
            return f"state {self.__name__}"
        param_overrides = getattr(self, "_param_overrides")
        params_str = ", ".join(f"{k}={v}" for k, v in param_overrides.items())
        return f"state {self.__qualname__} with {params_str}"

    def __repr__(self) -> str:
        return f"<{self._rendered_repr()}>"

    def __getattribute__(self, name: str) -> Any:
        attr = super().__getattribute__(name)
        if (
            isinstance(attr, type)
            and issubclass(attr, Outcome)
            and hasattr(attr, "_state")
            and attr._state is not self
        ):
            warnings.warn(
                f"{attr} accessed through {self}. This may lead to unexpected behavior when used in a transiton map.",
                stacklevel=2,
            )
        return attr


class State(metaclass=StateMeta):
    """States contain logic that will be executed by the StateMachine.

    Each State also contains class variables for each parameter on the parameter server, which
        can be accessed using self. Data that should be shared between calls of handle should be
        set as an instance variable.
    The state also shares the captain node, which can be used for io
    """

    _num_unexpected_params = 0

    def __init__(self, prev_outcome: Outcome, node: PIO):
        """
        node: The io_node which can be used via the Periodic_IO interface to access various publishers
        and subscribers, and can be used to create new publishers/subscribers
        """
        self.prev_outcome = prev_outcome
        self.io_node = node

    @abstractmethod
    def handle(self) -> Optional[Outcome]:
        """Contains the logic to be run for a particular state.

        Is called repeatedly for each iteration of the state, including the first one.
        """
        pass

    @classmethod
    def is_valid_income_type(cls, outcome_type: Type[Outcome]) -> bool:
        return True

    @classmethod
    def with_params(cls, **kwargs: Any) -> Type[Self]:
        num_unexpected = 0
        for k in kwargs:
            if not hasattr(cls, k):
                warnings.warn(
                    f"overriding parameter {k}, which is not defined on {cls}",
                    stacklevel=2,
                )
                num_unexpected += 1
        overrides: dict = getattr(cls, "_param_overrides", {}).copy()
        overrides.update(kwargs)
        kwargs["_param_overrides"] = overrides
        kwargs["__module__"] = cls.__module__
        kwargs["_num_unexpected_params"] = cls._num_unexpected_params + num_unexpected
        return type(cls.__name__, (cls,), kwargs)

    @classmethod
    def __repr__(cls) -> str:
        return f"<instance of {cls._rendered_repr()}>"


TransitionMap = Dict[Type[Outcome], Type[State]]


class StateMachine:
    """The main interface for running a system."""

    def __init__(
        self,
        name: str,
        transitions: TransitionMap,
        StartState: Type[State],
        StopState: Type[State],
        captainNode: PIO
    ):
        """Creates a new state machine.

        You should call this code once for any particular run.
        name: a string representing the name of the machine (used for overriding default parameters)
        transitions: a dictionary from Outcomes to States. Each of these should be the class itself, rather
                than an instance of the class
        StartState: the class of the state to begin with
        StopState: the class of the state to end with. when this state is reach, its handle will be
            called once, then the run method will return.
        captainNode: the captain node from which all the publishers/subscribers and other ros things for the
            various states are built off of
        """
        self.name = name
        self.StartState = StartState
        self.transitions = transitions
        self.StopState = StopState
        self.node = captainNode

        self._soft_stop_srv = self.node.create_service(Trigger, SOFT_STOP_SERVICE, self.soft_stop)
        self.stop_signal_recvd = False

    def soft_stop(self, data) -> Tuple[bool, str]:
        self.stop_signal_recvd = True
        return True, type(self.current_state).__qualname__

    def run(self, hz: int = 50) -> Optional[Outcome]:
        """Performs a run, beginning with the StartState and ending when it reaches StopState.

        Returns the Outcome from calling handle() on StopState.
        """
        rate = self.node.create_rate(hz)
        publisher = self.node.create_publisher(String, STATE_TOPIC, 1)
        self.current_state = self.StartState(InitTransition(), self.node)
        while type(self.current_state) != self.StopState:
            self.run_once(publisher)
            rate.sleep()
        msg = String()
        msg.data = type(self.current_state).__qualname__
        publisher.publish(msg)
        return self.current_state.handle()

    def run_once(self, state_topic_pub: rclpy.publisher.Publisher) -> None:
        """Runs one iteration of the state machine"""
        msg = String()
        msg.data = type(self.current_state).__qualname__
        state_topic_pub.publish(msg)

        outcome = self.current_state.handle()
        if self.stop_signal_recvd:
            if outcome is None:
                outcome = SoftStopTransition()
            NextState = self.StopState
            outcome_name = "!! Abort !!"
            self.node.get_logger().info(
                f"Aborting from state {type(self.current_state).__qualname__} and moving to stop state"
            )
        else:
            if outcome is None:
                return
            outcome_type = type(outcome)
            outcome_name = outcome_type.__qualname__
            NextState = self.transitions[outcome_type]

            if type(self.current_state) == NextState:
                self.node.get_logger().warn(
                    f"{type(self.current_state).__qualname__} contains a type which returns itself!"
                )

        self.node.get_logger().info(
            f"transition {type(self.current_state).__qualname__} --[{outcome_name}]--> {NextState.__qualname__}"
        )
        self.current_state = NextState(outcome, self.node)
