import rospy
from state_machine import *


class Start(State):
    Complete = Outcome.make('Complete')

    def initialize(self, prev_outcome: Outcome) -> None:
        print('Startup...')

    def handle(self) -> Outcome:
        return self.Complete()


class Submerge(State):
    Timeout = Outcome.make('Timeout')
    ReachedTarget = Outcome.make('ReachedTarget')
    FailedToReachTarget = Outcome.make('FailedToReachTarget')

    def initialize(self, prev_outcome: Outcome) -> None:
        self.depth = 0

    def handle(self) -> Outcome:
        if self.depth == 10:
            return self.ReachedTarget()
        else:
            self.depth += 1
            return self.FailedToReachTarget()


class Turn(State):
    Timeout = Outcome.make('Timeout')
    ReachedTarget = Outcome.make('ReachedTarget')
    FailedToReachTarget = Outcome.make('FailedToReachTarget')

    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.time()

    def handle(self) -> Outcome:
        return self.Timeout()


class Stop(State):
    Success = Outcome.make('Success', outcome=None)
    Failure = Outcome.make('Failure', outcome=None)

    def initialize(self, prev_outcome: Outcome) -> None:
        print('Teardown...')
        self.outcome = prev_outcome
        self.error = type(prev_outcome) in (Submerge.Timeout, Turn.Timeout)

    def handle(self) -> Outcome:
        return self.Failure(self.outcome) if self.error else self.Success(self.outcome)


if __name__ == '__main__':
    transitions = {
        Start.Complete: Submerge,
        Submerge.ReachedTarget: Turn,
        Submerge.FailedToReachTarget: Submerge,
        Submerge.Timeout: Stop,
        Turn.ReachedTarget: Stop,
        Turn.FailedToReachTarget: Turn,
        Turn.Timeout: Stop,
    }
    state_machine = StateMachine('Example', transitions, Start, Stop)
    outcome = state_machine.run(None)
    print(outcome)
