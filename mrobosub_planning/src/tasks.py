import rospy
from periodic_io import PIO
from state_machine import *


class Turn(State):
    Timeout = Outcome.make('Timeout')
    ReachedTarget = Outcome.make('ReachedTarget')
    InProgress = Outcome.make('InProgress')

    timeout_time: Param[float]
    target_yaw: Param[float]
    tolerance: Param[float]

    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()

    def handle(self) -> Outcome:
        if self.at_target():
            return self.ReachedTarget()
        elif rospy.get_time() - self.start_time >= self.timeout_time:
            return self.Timeout()
        else:
            PIO.yaw_pub.publish(1)
            return self.InProgress()

    def at_target(self) -> bool:
        return self.target_yaw - self.tolerance <= PIO.curr_yaw <= self.target_yaw + self.tolerance


class Submerge(State):
    Timeout = Outcome.make('Timeout')
    ReachedTarget = Outcome.make('ReachedTarget')
    InProgress = Outcome.make('InProgress')

    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()

    def handle(self) -> Outcome:
        if self.at_target():
            return self.ReachedTarget()
        elif rospy.get_time() - self.start_time >= self.timeout_time:
            return self.Timeout()
        else:
            PIO.heave_pub.publish(1)
            return self.InProgress()

    def at_target(self) -> bool:
        return self.target_heave - self.tolerance <= PIO.curr_heave <= self.target_heave + self.tolerance
