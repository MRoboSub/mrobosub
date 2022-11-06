import rospy
from periodic_io import PIO
from state_machine import *


class Submerge(State):
    Timeout = Outcome.make('Timeout')
    ReachedTarget = Outcome.make('ReachedTarget')
    FailedToReachTarget = Outcome.make('FailedToReachTarget')

    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.time()

    def handle(self) -> Outcome:
        if self.at_target():
            return self.ReachedTarget()
        elif rospy.time() - self.start_time >= self.timeout_time:
            return self.Timeout()
        else:
            PIO.curr_heave += 1
            return self.ReachedTarget()

    def at_target(self) -> bool:
        return self.target_heave - self.tolerance <= PIO.curr_heave <= self.target_heave + self.tolerance
