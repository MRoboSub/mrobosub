"""Contains the state machine implementation. You probably shouldn't read this unless you want to deal with wierd
Python metaprogramming."""


from __future__ import annotations
from abc import ABC, ABCMeta, abstractmethod
from dataclasses import make_dataclass, field
from typing import Mapping, Type, Final, cast
import rospy
from std_msgs.msg import String

from copy import copy


STATE_TOPIC = 'current_state'


Param = Final


__all__ = ('Outcome', 'State', 'StateMachine', 'Param')


class Outcome(ABC):
    """Abstract base classes for state outcomes.

    Outcomes are used to decide which state to run next. This decision is based only on the type of outcome, and not on
    any data that the outcome contains. In the following example, instances of outcomeA map to the same State, while
    outcomeB will map to a different state.

    Example:
        >>> OutcomeA = Outcome.make('OutcomeA', offset=0)
        >>> OutcomeB = Outcome.make('OutcomeB')
        >>> outcome_a_1 = OutcomeA(offset=10)
        >>> outcome_a_2 = OutcomeA(offset=-5)
        >>> outcome_b = OutcomeB()
    """

    @classmethod
    def make(cls, name: str, **kwargs: Mapping[str, Type]) -> Type[Outcome]:
        """Returns a dataclass type (not an instance of a class, but a new class type) which is a subclass of Outcome
        and has the default variables specified in **kwargs.

        Args:
            name: The name of the outcome; should always be the same as the variable it's assigned to.

        Example:
            >>> ReachedTargetOutcome = Outcome.make('ReachedTargetOutcome', duration=0)
            >>> current_outcome = ReachedTargetOutcome(duration=10)
            >>> current_outcome
            'ReachedTargetOutcome(duration=10)'
            >>> current_outcome = ReachedTargetOutcome()
            'ReachedTargetOutcome(duration=0)'
        """
        fields = [(key, val, field()) for key, val in kwargs.items()]
        dataclass = make_dataclass(name, fields, bases=(cls, ))
        return cast(Type[Outcome], dataclass)  # purely for better type hinting, no actual effect


class StateMeta(ABCMeta):
    def __new__(mcls: type[StateMeta], name: str, bases: tuple[type, ...], namespace: dict[str, Any], **kwargs: Any) -> StateMeta:
        try:
            for var, type_ in namespace["__annotations__"].items():
                if type_ is Outcome:
                    namespace[var] = Outcome.make(name + var)
        except AttributeError:
            pass
        return super().__new__(name, bases, namespace, **kwargs)


class State(ABC):
    """States contain logic that will be executed by the StateMachine."""

    def __init__(self, prev_outcome: Outcome):
        self.initialize(prev_outcome)

    @abstractmethod
    def initialize(self, prev_outcome: Outcome) -> None:
        """Runs when the state is initialized.

        This is run whenever the same state isn't run consecutively, for example in the following flow initialize is
        run for states prefixed with *:

            *Start -> *StateA -> StateA -> *StateB -> StateB -> *StateA -> *End
        """
        pass

    @abstractmethod
    def handle(self) -> Outcome:
        """Contains the logic to be run for a particular state.

        Unlike initialize which will only run once, if the same state is run consecutively handle is run whenever
        however many times the state is run.
        """
        pass


class StateMachine:
    def __init__(self, name: str, transitions: Mapping[Type[Outcome], Type[State]], StartState: Type[State], StopState: Type[State]):
        self.name = name
        self.StartState = StartState
        self.transitions = transitions
        self.StopState = StopState

        self._load_params(State, 'globals', 'defaults')
        self._load_params(State, 'globals', self.name)
        states = set(transitions.values()).union({self.StartState})
        for state in states:
            self._load_params(state, state.__module__, 'mod', 'defaults')
            self._load_params(state, state.__module__, 'mod', self.name)
            self._load_params(state, state.__module__, state.__name__.lower(), 'defaults')
            self._load_params(state, state.__module__, state.__name__.lower(), self.name)

    @staticmethod
    def _load_params(state: Type[State], module: str, state_name, machine_name: str = '') -> None:
        """Set all parameters in the module/state_name/machine_name namespace of the ROS parameter services as class
        variables in state."""
        if module == '__main__':
            module = 'globals'
        namespace = f'~{module}/{state_name}/{machine_name}' if machine_name else f'~{module}/{state_name}'
        print(f'loading namespace {namespace}')
        for key, value in rospy.get_param(namespace, {}).items():
            print(f'\t{key}: {value}')
            setattr(state, key, property(lambda self: value))  # read-only constants

    def run(self) -> Outcome:
        publisher = rospy.Publisher(STATE_TOPIC, String, queue_size=1)
        current_state = self.StartState(None)
        while type(current_state) != self.StopState:
            publisher.publish(type(current_state).__qualname__)
            outcome = current_state.handle()
            outcome_type = type(outcome) if isinstance(outcome, Outcome) else outcome
            NextState = self.transitions[outcome_type]
            rospy.logdebug(f'{type(current_state).__qualname__} -> {NextState.__qualname__}')
            if type(current_state) != NextState:
                current_state = NextState(outcome)
                rospy.loginfo(f'transition {type(current_state).__qualname__} -> {NextState.__qualname__}')
        publisher.publish(type(current_state).__qualname__)
        return current_state.handle()  # handle stop state
        
