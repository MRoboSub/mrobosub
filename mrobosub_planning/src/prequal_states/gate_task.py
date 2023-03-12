from state_machine import Outcome, TimedState, State, Param
from periodic_io import PIO, angle_error

class AlignGate(TimedState):
    Unaligned = Outcome.make('Unaligned')
    ReachedAngle = Outcome.make('ReachedAngle')
    TimedOut = Outcome.make("TimedOut")

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[int]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_theshold):
            return self.ReachedAngle()
        else:
            return self.Unaligned()
    
    def handle_once_timedout(self) -> None:
        PIO.set_target_pose_yaw(0)
        
class ApproachGate(State):
    Unreached = Outcome.make('Unreached')
    Reached = Outcome.make("Reached")
    
    target_surge_time: Param[float]
    speed: Param[float]
    start_time: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(self.speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            return self.Reached()
        else:
            self.Unreached()
        

class ApproachMarker(State):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")

    target_surge_time: Param[float]
    speed: Param[float]
    start_time: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(self.speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            return self.Reached()
        else:
            self.Unreached()

class MoveAroundMarker(State):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")

    target_sway_time: Param[float]
    speed: Param[float]
    start_time: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_sway(self.speed)
        if rospy.get_time() - self.start_time >= self.target_sway_time:
            return self.Reached()
        else:
            self.Unreached()

class ReturnGate(State):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")

    target_surge_time: Param[float]
    speed: Param[float]
    start_time: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(-self.speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            return self.Reached()
        else:
            self.Unreached()
    