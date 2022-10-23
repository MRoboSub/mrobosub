from typing import Mapping, Type
import rospy

class Outcome:
    def __hash__(self):
        return hash(str(self))

    def __str__(self):
        return self.__class__.__name__
    

def make_outcome(name, **default_fields):
    class SubOutcome(Outcome):
        def __init__(self, **kwargs):
            for df in default_fields:
                setattr(self, df, default_fields[df])

            for kw in kwargs:
                setattr(self, kw, kwargs[kw])

    SubOutcome.__name__ = name
    return SubOutcome


class State:
    last_handled_state = None

    def __init__(self, name: str, HandlerType: Type[StateHandler]):
        self.HandlerType = HandlerType

    def handle(self, outcome: Outcome, gbl_ctx: Context):
        if not last_handled_state is self:
            self.handler = self.HandlerType.__init__(outcome, gbl_ctx)

        State.last_handled_state = self
        return self.handler.iteration(gbl_ctx)

    def __eq__(self, other):
        return self.name == other.name


class Context:
    def __init__(self, param_name):
        params = rospy.getparam(param_name)
        for key in params:
            setattr(self, params[key])

class StateHandler:
    def __init__(self, prev_outcome: Outcome, gbl_ctx: Context):
        pass

    def iteration(self, gbl_ctx: Context):
        pass


class StateMachine:
    def __init__(self, transitions: Mapping[Outcome, State],
            initial_state: State, stop_state: State):
        # TODO: should this take instances or types?
        self.curr_state = initial_state
        self.success_state = success_state
        self.fail_state = fail_state
        self.transitions = transitions

    def run(self, gbl_ctx: Context):
        outcome = None
        while not (self.curr_state == self.stop_state):
            outcome = self.curr_state.handle(outcome, gbl_ctx)
            self.curr_state = self.transitions[outcome]


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



