import rospy
from state_machine import *
from periodic_io import PIO
from submerge import Submerge
from turn import Turn


class Start(State):
    Complete = Outcome.make('Complete')

    def initialize(self, prev_outcome: Outcome) -> None:
        print('Startup...')

    def handle(self) -> Outcome:
        return self.Complete()


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
    outcome = state_machine.run()
    print(outcome)
