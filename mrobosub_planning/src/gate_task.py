from umrsm import Outcome, TimedState, State, TurnToYaw
from periodic_io import PIO, angle_error, Glyph, Gbl
from buoy_task import SeenGlyph, search_for_glyph
from mrobosub_msgs.srv import ObjectPositionResponse
import rospy
from typing import NamedTuple, Union

class FoundPathMarker(NamedTuple):
    angle: float

class SeenGateImage(NamedTuple):
    position: ObjectPositionResponse
    glyph_seen: Glyph

class AlignGate(TimedState):
    class Unaligned(NamedTuple):
        pass
    class ReachedAngle(NamedTuple):
        pass 
    class TimedOut(NamedTuple):
        pass

    target_yaw: float = 0.0
    timeout: float = 10.0 
    yaw_threshold: float = 2.0

    def handle_if_not_timedout(self) -> Union[ReachedAngle, Unaligned]:
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.ReachedAngle()
        else:
            return self.Unaligned()

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_pose_yaw(0)
        
        return self.TimedOut()
    
class ApproachGate(TimedState):
    class Unreached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass
    
    timeout: float = 26.0
    surge_speed: float = 0.2

    def __init__(self, prev_outcome: NamedTuple):
        if type(prev_outcome) != SeenGateImage:
            raise TypeError(prev_outcome)
        super().__init__(prev_outcome)
        self.found_image_threshold = 50
        self.times_seen = 0

    def handle_if_not_timedout(self) -> Union[Unreached, SeenGateImage]:
        PIO.set_target_twist_surge(self.surge_speed)

        abydos_response = PIO.query_glyph(Glyph.abydos)
        earth_response = PIO.query_glyph(Glyph.earth)
        res_exists = abydos_response.found or earth_response.found

        if not res_exists:
            return self.Unreached()

        if self.times_seen >= self.found_image_threshold:
            if abydos_response.found:
                return SeenGateImage(position=abydos_response, glyph_seen=Glyph.abydos)
            else:
                return SeenGateImage(position=earth_response, glyph_seen=Glyph.earth)
    
        self.times_seen += 1
        return self.Unreached()
    
    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)

        return self.TimedOut()
    
# class Scan(TimedState):
#     NotFound = Outcome.make('NotFound')
#     TimedOut = Outcome.make('TimedOut')
#     FoundPathMarker = Outcome.make('FoundPathMarker')

#     timeout: int

#     def handle_if_not_timedout(self) -> Outcome:
#         if PIO.have_seen_pathmarker():
#             return self.FoundPathMarker()
#         else:
#             return self.NotFound()

#     def handle_once_timedout(self) -> Outcome:
#         return self.TimedOut()

class ApproachGateImage(TimedState):
    class GoneThroughGate(NamedTuple):
        planet: Glyph
    class TimedOut(NamedTuple):
        pass

    timeout: float = 25.0
    lost_image_threshold: int = 15
    yaw_threshold: float = 2.0

    def __init__(self, prev_outcome: NamedTuple) -> None:
        if type(prev_outcome) != SeenGateImage:
            raise TypeError(prev_outcome)
        super().__init__(prev_outcome)
        Gbl.planet_seen = prev_outcome.glyph_seen
        self.last_target_yaw = PIO.Pose.yaw + prev_outcome.position.x_theta
        self.lost_image_threshold = 300
        self.times_not_seen = 0

    def handle_if_not_timedout(self) -> Union[GoneThroughGate, SeenGateImage]:
        # Precondition: you have already seen a glyph. You are trying to update
        if type(Gbl.planet_seen) != Glyph:
            raise TypeError(Gbl.planet_seen)
        
        resp = PIO.query_glyph(Gbl.planet_seen)
        if not resp.found:
            self.times_not_seen += 1
            if self.times_not_seen >= self.lost_image_threshold:
                return self.GoneThroughGate(planet=Gbl.planet_seen)
        else:
            self.times_not_seen = 0
            self.last_target_yaw = PIO.Pose.yaw + resp.x_theta * 0.5

        #pm_angle = self.query_pathmarker()
        #if pm_angle is not None:
        #    return FoundBuoyPathMarker(angle=pm_angle)

        PIO.set_target_pose_yaw(self.last_target_yaw)
        return SeenGateImage(glyph_seen=Gbl.planet_seen, position=resp)        

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()

# class AlignPathMarker(TimedState):
#     class Unaligned(NamedTuple):
#         pass
#     class Aligned(NamedTuple):
#         pass
#     class TimedOut(NamedTuple):
#         pass

#     yaw_threshold: float = 2.0
#     timeout: int = 10

#     def __init__(self, prev_outcome: FoundBuoyPathMarker) -> None:
#         super().__init__(prev_outcome)
#         self.last_known_angle = prev_outcome.angle


#     def handle_if_not_timedout(self) -> Outcome:
#         pm_resp = PIO.query_pathmarker()
#         if pm_resp is not None:
#             self.last_known_angle = pm_resp

#         seen_glyph_outcome = search_for_glyph(Gbl.preferred_glyph())
        
#         PIO.set_target_pose_yaw(self.last_known_angle)

#         if seen_glyph_outcome:
#             return seen_glyph_outcome
#         elif PIO.is_yaw_within_threshold(self.yaw_threshold):
#             return self.Aligned()
#         else:
#             return self.Unaligned()
        
#     def handle_once_timedout(self) -> Outcome:
#         return self.TimedOut()
    
# class FallBackTurn(State):
#     Unaligned = Outcome.make('Unaligned')
#     Aligned = Outcome.make('Aligned', angle=float)   

#     target_yaw: float = 0.0
#     yaw_threshold: float = 2.0
   
#     def __init__(self, prev_outcome: FoundBuoyPathMarker) -> None:
#         super().__init__(prev_outcome)
 
#     def handle(self) -> Outcome:
#         PIO.set_target_pose_yaw(self.target_yaw)

#         seen_glyph_outcome = search_for_glyph(Gbl.preferred_glyph())

#         if seen_glyph_outcome:
#             return seen_glyph_outcome
#         elif PIO.is_yaw_within_threshold(self.yaw_threshold):
#             return self.Aligned()
#         else:
#             return self.Unaligned()

class Spin(TimedState):
    timeout: float = 30.0

    class Unreached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        self.timeout = 15.0

    def handle_if_not_timedout(self) -> Unreached:
        PIO.set_target_twist_yaw(0.12)
        PIO.set_target_pose_heave(1)
        return self.Unreached()

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_yaw(0)

        return self.TimedOut()
    
class SpinFinish(TimedState):
    class Unreached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    yaw_threshold: float = 2.0
    timeout: float = 30.0

    def handle_if_not_timedout(self):
        target_yaw = 0
        PIO.set_target_pose_yaw(target_yaw)

        if not PIO.is_yaw_within_threshold(5):
            self.timer = rospy.get_time()

        if rospy.get_time() - self.timer >= 1:
            reached = self.Reached()
            reached.angle = target_yaw
            return self.Reached()

        unreached = self.Unreached()
        unreached.angle = target_yaw
        return unreached

    def handle_once_timedout(self):
        return self.TimedOut()