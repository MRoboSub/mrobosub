from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import make_dataclass, field
from typing import Mapping, Type
import rospy


__all__ = ['Outcome', 'State', 'Context', 'StateMachine']


class Outcome(ABC):
    @classmethod
    def make(cls, name: str, **kwargs):
        fields = [(key, type(val), field(default=val)) for key, val in kwargs.items()]
        return make_dataclass(name, fields, bases=(cls, ))


class State(ABC):
    def __init__(self, prev_outcome: Outcome, gbl_ctx: Context):
        self.initialize(prev_outcome, gbl_ctx)

    @abstractmethod
    def initialize(self, prev_outcome: Outcome, gbl_ctx: Context) -> None:
        pass

    @abstractmethod
    def handle(self, gbl_ctx: Context) -> Outcome:
        pass


class Context:
    def __init__(self, param_name):
        params = rospy.getparam(param_name)
        for key in params:
            setattr(self, key, params[key])


class StateMachine:
    def __init__(self, transitions: Mapping[Type[Outcome], Type[State]], StartState: Type[State], StopState: Type[State]):
        self.StartState = StartState
        self.transitions = transitions
        self.StopState = StopState

    def run(self, gbl_ctx: Context) -> Outcome:
        current_state = self.StartState(None, gbl_ctx)
        while type(current_state) != self.StopState:
            outcome = current_state.handle(gbl_ctx)
            NextState = self.transitions[type(outcome)]
            if type(current_state) != NextState:
                current_state = NextState(outcome, gbl_ctx)
        return current_state.handle(gbl_ctx)  # handle stop state
