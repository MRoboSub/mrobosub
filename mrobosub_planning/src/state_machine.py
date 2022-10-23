from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import make_dataclass, field
from typing import Mapping, Type
# import rospy


class Outcome(ABC):
    def __hash__(self):
        return hash(type(self))

    def __str__(self):
        return self.__class__.__name__

    @classmethod
    def __new__(cls, *args, **kwargs):
        return make_dataclass(
            args[0],
            [(key, type(value), field(default=value)) for key, value in kwargs.items()],
            bases=(cls,),
        )
    

def make_outcome(name, **default_fields):
    return make_dataclass(
        name,
        [(key, type(value), field(default=value)) for key, value in default_fields.items()],
        bases=(Outcome, ),
    )


class SubState:
    outcome1 = None
    outcome2 = None


class State2(ABC):
    def __int__(self, prev_outcome: Outcome, gbl_ctx: Context):
        self.initialize(prev_outcome, gbl_ctx)

    @abstractmethod
    def initialize(self, prev_outcome: Outcome, gbl_ctx: Context):
        pass

    @abstractmethod
    def handle(self, gbl_ctx: Context):
        pass


class State:
    last_handled_state = None

    def __init__(self, name: str, HandlerType: Type[StateHandler]):
        self.HandlerType = HandlerType
        self.name = name

    def handle(self, outcome: Outcome, gbl_ctx: Context):
        if self.last_handled_state is not self:
            self.handler = self.HandlerType(outcome, gbl_ctx)

        State.last_handled_state = self
        return self.handler.iteration(gbl_ctx)

    def __eq__(self, other):
        return self.name == other.name


class Context:
    def __init__(self, param_name):
        params = []#rospy.getparam(param_name)
        for key in params:
            setattr(self, key, params[key])


class StateHandler(ABC):
    def __init__(self, prev_outcome: Outcome, gbl_ctx: Context):
        self.prev_outcome = prev_outcome
        self.gbl_ctx = gbl_ctx

    @abstractmethod
    def iteration(self, gbl_ctx: Context):
        pass


class StateMachine:
    def __init__(self, transitions: Mapping[Type[Outcome], Type[State2]], StartState: Type[State2], StopState: Type[State2]):
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


if __name__ == '__main__':
    Timeout = make_outcome('Timeout')
    NotThere = make_outcome('NotThere', error=15)
    ReachDepth = make_outcome('ReachDepth', depth=78)

    to = Timeout()
    nt = NotThere(error=12)
    rd = ReachDepth()

    print(to)
    print(nt.__dict__)
    print(nt)
    print(rd.__dict__)



