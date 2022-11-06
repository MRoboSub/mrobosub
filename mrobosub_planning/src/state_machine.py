from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import make_dataclass, field
from typing import Mapping, Type
import rospy


__all__ = ['Outcome', 'State', 'StateMachine']


class Outcome(ABC):
    @classmethod
    def make(cls, name: str, **kwargs):
        """Returns a dataclass type (not an instance of a class, but a new class) which is a subclass of Outcome and has
        the default variables specified in **kwargs."""
        fields = [(key, type(val), field(default=val)) for key, val in kwargs.items()]
        return make_dataclass(name, fields, bases=(cls, ))


class State(ABC):
    def __init__(self, prev_outcome: Outcome):
        self.initialize(prev_outcome)

    @abstractmethod
    def initialize(self, prev_outcome: Outcome) -> None:
        pass

    @abstractmethod
    def handle(self) -> Outcome:
        pass


class StateMachine:
    def __init__(self, name: str, transitions: Mapping[Type[Outcome], Type[State]], StartState: Type[State], StopState: Type[State]):
        self.name = name
        self.StartState = StartState
        self.transitions = transitions
        self.StopState = StopState

        # TODO: make readonly
        self._load_params(State, 'globals', 'defaults')
        self._load_params(State, 'globals', self.name)
        states = set(transitions.values()).union({self.StartState})
        for state in states:
            self._load_params(state, state.__module__, 'mod', 'defaults')
            self._load_params(state, state.__module__, 'mod', self.name)
            self._load_params(state, state.__module__, state.__name__.lower(), 'defaults')
            self._load_params(state, state.__module__, state.__name__.lower(), self.name)

    @staticmethod
    def _load_params(state: Type, module: str, state_name, machine_name: str = ''):
        """Set all parameters in the module/state_name/machine_name namespace of the ROS parameter services as class
        variables in state."""
        namespace = f'{module}/{state_name}/{machine_name}' if machine_name else f'{module}/{state_name}'
        for key, value in rospy.get_param(namespace, {}).items():
            setattr(state, key, value)

    def run(self) -> Outcome:
        current_state = self.StartState(None)
        while type(current_state) != self.StopState:
            outcome = current_state.handle()
            # TODO: automatically convert Outcome to Outcome() if necessary
            NextState = self.transitions[type(outcome)]
            if type(current_state) != NextState:
                current_state = NextState(outcome)
        return current_state.handle()  # handle stop state
