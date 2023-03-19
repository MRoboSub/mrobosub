import rospy
from state_machine import *
from periodic_io import PIO

class Submerge(State):
    submerge_depth: Param[float]
    timeout_time: Param[int]

    TimedOut = Outcome.make('TimedOut')
    ReachedDepth = Outcome.make('ReachedDepth')
    SubmergeAgain = Outcome.make("SubmergeAgain")
    
    def initialize(self, prev_outcome: Outcome) -> None:
        print('SUBMERGE', self.submerge_depth)
        print('TIMEOUT', self.timeout_time)
        self.start_time = rospy.get_time()


    def handle(self) -> Outcome:
        """ Submerges to target depth """
        PIO.set_target_pose_heave(self.submerge_depth)
        
        if abs(PIO.get_pose().heave - self.submerge_depth) <= 0.1:
            return self.ReachedDepth()
        elif rospy.get_time() - self.start_time > self.timeout_time:
            return self.TimedOut()
        else:
            return self.SubmergeAgain()