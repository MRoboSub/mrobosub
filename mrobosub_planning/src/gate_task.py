from umrsm import Outcome, TimedState, State, Param, TurnToYaw
from periodic_io import PIO, angle_error, Glyph, Gbl
from buoy_task import SeenGlyph, search_for_glyph
from mrobosub_msgs.srv import ObjectPositionResponse

FoundBuoyPathMarker = Outcome.make('FoundPathMarker', angle=float)
SeenGateImage = Outcome.make('SeenGateImage', position=ObjectPositionResponse, glyph_seen=Glyph)

class AlignGate(TimedState):
    Unaligned = Outcome.make('Unaligned')
    ReachedAngle = Outcome.make('ReachedAngle')
    TimedOut = Outcome.make('TimedOut')

    target_yaw: Param[float]
    timeout: Param[int]
    yaw_threshold: Param[float]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.ReachedAngle()
        else:
            return self.Unaligned()

    def handle_once_timedout(self) -> None:
        PIO.set_target_pose_yaw(0)

class ApproachGate(TimedState):
    Unreached = Outcome.make('Unreached')
    TimedOut = Outcome.make('TimedOut')
    
    timeout: Param[int]
    surge_speed: Param[float]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_twist_surge(self.surge_speed)
        pm_angle = PIO.query_pathmarker()

        abydos_response = PIO.query_glyph(Glyph.abydos)
        earth_response = PIO.query_glyph(Glyph.earth)

        if pm_angle is not None:
            return FoundBuoyPathMarker(angle=pm_angle)
        elif abydos_response.found:
            return SeenGateImage(position=abydos_response, glyph_seen=Glyph.abydos)
        elif earth_response.found:
            return SeenGateImage(position=earth_response, glyph_seen=Glyph.earth)
        else:
            return self.Unreached()
    
    def handle_once_timedout(self) -> None:
        PIO.set_target_twist_surge(0)

    
# class Scan(TimedState):
#     NotFound = Outcome.make('NotFound')
#     TimedOut = Outcome.make('TimedOut')
#     FoundPathMarker = Outcome.make('FoundPathMarker')

#     timeout: Param[int]

#     def handle_if_not_timedout(self) -> Outcome:
#         if PIO.have_seen_pathmarker():
#             return self.FoundPathMarker()
#         else:
#             return self.NotFound()

class ApproachGateImage(TimedState):
    GoneThroughGate = Outcome.make('GoneThroughGate', planet=Glyph)
    TimedOut = Outcome.make('TimedOut')

    timeout: Param[float]
    lost_image_threshold: Param[int]
    yaw_threshold: Param[float]

    def initialize(self, prev_outcome: SeenGateImage) -> None:
        super().initialize(prev_outcome)
        Gbl.planet_seen = prev_outcome.glyph_seen
        self.last_target_yaw = PIO.Pose.yaw + prev_outcome.position.x_theta
        self.times_not_seen = 0

    def handle_if_not_timedout(self) -> Outcome:
        resp = PIO.query_glyph(Gbl.planet_seen)
        if not resp.found:
            self.times_not_seen += 1
            if self.times_not_seen >= self.lost_image_threshold:
                return self.GoneThroughGate(planet=Gbl.planet_seen)
        else:
            self.times_not_seen = 0
            self.last_target_yaw = PIO.Pose.yaw + resp.x_theta

        #pm_angle = self.query_pathmarker()
        #if pm_angle is not None:
        #    return FoundBuoyPathMarker(angle=pm_angle)

        PIO.set_target_pose_yaw(self.last_target_yaw)
        return SeenGateImage(glyph_seen=Gbl.planet_seen, position=resp)        


class AlignPathMarker(TimedState):
    Unaligned = Outcome.make('Unaligned')
    Aligned = Outcome.make('Aligned')
    TimedOut = Outcome.make('TimedOut')

    yaw_threshold: Param[float]
    timeout: Param[int]

    def initialize(self, prev_outcome: FoundBuoyPathMarker) -> None:
        super().initialize(prev_outcome)
        self.last_known_angle = prev_outcome.angle


    def handle_if_not_timedout(self) -> Outcome:
        pm_resp = PIO.query_pathmarker()
        if pm_resp is not None:
            self.last_known_angle = pm_resp

        seen_glyph_outcome = search_for_glyph(Gbl.preferred_glyph())
        
        PIO.set_target_pose_yaw(self.last_known_angle)

        if seen_glyph_outcome:
            return seen_glyph_outcome
        elif PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Aligned()
        else:
            return self.Unaligned()

class FallBackTurn(State):
    Unaligned = Outcome.make('Unaligned')
    Aligned = Outcome.make('Aligned', angle=float)   

    target_yaw: Param[float]
    yaw_threshold: Param[float]
   
    def initialize(self, prev_outcome: FoundBuoyPathMarker) -> None:
        super().initialize(prev_outcome)
 
    def handle(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)

        seen_glyph_outcome = search_for_glyph(Gbl.preferred_glyph())

        if seen_glyph_outcome:
            return seen_glyph_outcome
        elif PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Aligned()
        else:
            return self.Unaligned()

class Spin(TimedState):
    timeout: Param[float]

    Unreached = Outcome.make('Unreached')
    TimedOut = Outcome.make('TimedOut')

    def initialize(self, prev_outcome: Outcome) -> None:
        super().initialize(prev_outcome)

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_twist_yaw(0.15)
        return self.Unreached        
    
    def handle_once_timedout(self) -> None:
        PIO.set_target_twist_yaw(0)
        return self.TimedOut()

class SpinFinish(TurnToYaw):
    timeout: Param[float]

    Unreached = Outcome.make('Unreached')
    Reached = Outcome.make('Reached')
    TimedOut = Outcome.make('TimedOut')

    target_yaw : Param[float]
    yaw_threshold: Param[float]
    settle_time: Param[float]
    timeout : Param[float]