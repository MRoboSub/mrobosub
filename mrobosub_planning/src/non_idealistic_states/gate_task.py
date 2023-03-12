from state_machine import Outcome, TimedState, State, Param
from periodic_io import PIO, angle_error

class AlignGate(TimedState):
    Unaligned = Outcome.make('Unaligned')
    ReachedAngle = Outcome.make('ReachedAngle')
    TimedOut = Outcome.make('TimedOut')

    target_yaw: Param[float]
    timeout: Param[int]
    yaw_threshold: Param[float]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_theshold):
            return self.ReachedAngle()
        else:
            return self.Unaligned()
    
    def handle_once_timedout(self) -> None:
        PIO.set_target_pose_yaw(0)

class ApproachGate(TimedState):
    Unreached = Outcome.make('Unreached')
    FoundPathMarker = Outcome.make("FoundPathMarker")
    TimedOut = Outcome.make('TimedOut')
    
    timeout: Param[int]
    speed: Param[float]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_twist_surge(self.speed)
        if PIO.have_seen_pathmarker():
            return self.FoundPathMarker()
        else:
            self.Unreached()
    
    def handle_once_timedout(self) -> None:
        PIO.set_target_twist_surge(0)

    
# class Scan(TimedState):
#     NotFound = Outcome.make("NotFound")
#     TimedOut = Outcome.make("TimedOut")
#     FoundPathMarker = Outcome.make("FoundPathMarker")

#     timeout: Param[int]

#     def handle_if_not_timedout(self) -> Outcome:
#         if PIO.have_seen_pathmarker():
#             return self.FoundPathMarker()
#         else:
#             return self.NotFound()

class AlignPathMarker(TimedState):
    Unaligned = Outcome.make("Unaligned")
    Aligned = Outcome.make("Aligned")

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    timeout: Param[int]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)
        if PIO.is_yaw_within_threshold(self.yaw_theshold):
            return self.Aligned()
        else:
            return self.Unaligned()

class FallBackTurn(State):
    Unaligned = Outcome.make("Unaligned")
    Aligned = Outcome.make("Aligned")   

    target_yaw: Param[float]
    yaw_threshold: Param[float]
    
    def handle(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)
        if PIO.is_yaw_within_threshold(self.yaw_theshold):
            return self.Aligned()
        else:
            return self.Unaligned()