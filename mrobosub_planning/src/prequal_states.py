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
    
        
class ApproachGate(State):
    Unreached = Outcome.make('Unreached')
    Reached = Outcome.make("Reached")
 
    target_surge_time: Param[float]
    speed: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(self.speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            PIO.set_target_twist_surge(0)
            return self.Reached()
        else:
            return self.Unreached()

        
# Concerns: -drift during submerge, poor movement on sway &surge axes, drift during surge.

class ApproachMarker(State):
    Unreached = Outcome.make("Unreached")
    Reached_0 = Outcome.make("Reached_0")
    Reached_1 = Outcome.make("Reached_1")

    movement_type: Param[int] #0: sway; 1: turn movem
    target_surge_time: Param[float]
    speed: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(self.speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            if self.movement_type == 0:
                PIO.set_target_twist_surge(0)
                return self.Reached_0()
            if self.movement_type == 1:
                PIO.set_target_twist_surge(0)
                return self.Reached_1()
        else:
            return self.Unreached()

class StrafeAroundMarker(State):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")

    target_sway_time: Param[float]
    sway_speed: Param[float]
        
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_sway(self.sway_speed)
        if rospy.get_time() - self.start_time >= self.target_sway_time:
            PIO.set_target_twist_sway(0)
            return self.Reached()
        else:
            return self.Unreached()
        
            
class ReturnGate(State):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")

    target_surge_time: Param[float]
    speed: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(-self.speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            PIO.set_target_twist_surge(0)
            return self.Reached()
        else:
            return self.Unreached()

class TurnAroundMarker(TimedState):
    Unreached = Outcome("Unreached")
    Reached = Outcome("Reached")
    TimedOut = Outcome("TimedOut")

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[int]
    
    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)
        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Reached()
        else:
            return self.Unreached()
    
        
class MovePastMarker(State):
    Unreached = Outcome.make("MovePastMarker")
    Reached = Outcome.make("TurnToGate")
    
    target_surge_time: Param[float]
    speed: Param[float]

    def initialize(self, prev_outcome: Outcome)-> None:
        self.start_time = rospy.get_time()
        
    def handle(self)->Outcome:
        PIO.set_target_twist_surge(self.speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            PIO.set_target_twist_surge(0)
            return self.Reached()
        else:
            return self.Unreached()
        
       
class TurnToGate(TimedState):
    Unreached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")
    TimedOut = Outcome.make("TimedOut")

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[int]
    
    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)
        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Reached()
        else:
            return self.Unreached()
        
class ReturnToGate(State):
    Unreached = Outcome.make("ReturnToGate")
    Reached = Outcome.make("Stop")
    
    target_surge_time: Param[float]
    speed: Param[float]
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        PIO.set_target_twist_surge(self.speed)
        if rospy.get_time() - self.start_time >= self.target_surge_time:
            PIO.set_target_twist_surge(0)
            return self.Reached()
        else:
            return self.Unreached()
    