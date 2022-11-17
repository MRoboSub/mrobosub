#! /usr/bin/env python

import rospy
from state_machine import *
from periodic_io import PIO
from doc.tasks import Turn, Submerge

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
        Submerge.InProgress: Submerge,
        Submerge.Timeout: Stop,
        Turn.ReachedTarget: Stop,
        Turn.InProgress: Turn,
        Turn.Timeout: Stop,
    }
    rospy.init_node('example')
    state_machine = StateMachine('example', transitions, Start, Stop)
    outcome = state_machine.run()
    print(outcome)
