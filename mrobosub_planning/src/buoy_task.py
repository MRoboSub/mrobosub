from umrsm import Outcome, TimedState, State, Param
from periodic_io import PIO, Gbl, angle_error, Glyph
from mrobosub_msgs.srv import ObjectPositionResponse
from typing import Mapping, NamedTuple, Union, Type

class SeenGlyph(NamedTuple):
    glyph_results: Mapping[Glyph, ObjectPositionResponse]
    glyph: Glyph

class HitBuoyFirst(NamedTuple):
    pass

class HitBuoySecond(NamedTuple):
    pass


def search_for_glyph(preference):
    res = PIO.query_all_glyphs()
    for g in [preference, Glyph.taurus, Glyph.serpens_caput, Glyph.auriga, Glyph.cetus]:
        if g in res:
            return SeenGlyph(glyph_results=res, glyph=g)
    else:
        return None

def first_glyph():
    if Gbl.planet_seen == Glyph.abydos:
        return Glyph.taurus
    else:
        return Glyph.auriga


class ApproachBuoyOpen(TimedState):
    class GlyphNotSeen(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass
    
    surge_speed: Param[float]
    m_timeout: Param[float]

    def initialize(self, prev_outcome) -> None:
        super().initialize(prev_outcome)
        if hasattr(prev_outcome, 'angle'):
            self.target_yaw = prev_outcome.angle
        else:
            self.target_yaw = PIO.Pose.yaw

    def handle_if_not_timedout(self) -> Union[SeenGlyph, GlyphNotSeen]:
        PIO.set_target_pose_yaw(self.target_yaw)
        PIO.set_target_twist_surge(self.surge_speed)

        seen_glyph_outcome = search_for_glyph(Gbl.preferred_glyph())
        # hit = PIO.buoy_collision

        if seen_glyph_outcome:
            # possible change: add some counter to increase the likelihood that we get our preferred glyph?
            return seen_glyph_outcome
        # elif hit:
        #     return HitBuoyFirst()
        else:
            return self.GlyphNotSeen()

    def handle_once_timedout(self) -> TimedOut:        
        PIO.set_target_twist_surge(0)
        return self.TimedOut()
    
    def timeout(self) -> float:
        return self.m_timeout


class CenterHeaveGlyph(TimedState):
    class NotCentered(NamedTuple):
        pass
    class Centered(NamedTuple):
        glyph: Glyph
        last_data: ObjectPositionResponse
    class TimedOut(NamedTuple):
        glyph: Glyph
        last_data: ObjectPositionResponse

    heave_down_speed: Param[float] 
    heave_up_speed: Param[float]
    deadband: Param[int] # pixels
    m_timeout: Param[float]


    def initialize(self, prev_outcome) -> None:
        super().initialize(prev_outcome)
        if type(prev_outcome) != SeenGlyph:
            raise TypeError(type(prev_outcome))
        self.deadband = 50
        self.m_timeout = 40.0
        self.most_recent_results = prev_outcome.glyph_results
        self.glyph = prev_outcome.glyph
        self.glyph_y_diff = self.most_recent_results[self.glyph].y_position

    def handle_if_not_timedout(self) -> Union[Centered, NotCentered]:
        query_res = PIO.query_glyph(self.glyph)
        if query_res.found:
            self.glyph_y_diff = query_res.y_position
            self.most_recent_results[self.glyph] = query_res

        # use updated info if we have it, otherwise continue with old info? is this bad
        if abs(self.glyph_y_diff) < self.deadband:
            return self.Centered(self.glyph, self.most_recent_results[self.glyph])
        elif self.glyph_y_diff > 0: # TODO: check sign?

            PIO.set_target_twist_heave(0.17)
        else:
            PIO.set_target_twist_heave(0)

        return self.NotCentered()

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut(self.glyph, self.most_recent_results[self.glyph])
    
    def timeout(self) -> float:
        return self.m_timeout



class CenterYawGlyph(TimedState):
    class NotReached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    surge_speed: Param[float]
    yaw_factor: Param[float]
    m_timeout: Param[float]

    def initialize(self, prev_outcome):
        if type(prev_outcome) != CenterHeaveGlyph.Centered:
            raise TypeError(type(prev_outcome))
        self.surge_speed = 0.2
        self.yaw_factor = 0.004
        self.m_timeout = 40
        super().initialize(prev_outcome)
        self.glyph = prev_outcome.glyph
        self.angle_diff = prev_outcome.last_data.x_theta
        self.target_heave = PIO.Pose.heave

    def handle_if_not_timedout(self) -> Outcome:
        
        print(self.glyph)
        query_res = PIO.query_glyph(self.glyph)
        if query_res.found:
            self.angle_diff = query_res.x_theta

        PIO.set_target_pose_heave(self.target_heave)
        PIO.set_target_twist_surge(self.surge_speed)

        # Use setpoint for yaw angle
        PIO.set_target_twist_yaw(self.angle_diff * self.yaw_factor)
        
        hit = PIO.buoy_collision
        
        if hit:
            if not Gbl.second_glpyh:
                Gbl.second_glpyh = True
                return HitBuoyFirst()
            else:
                return HitBuoySecond()
        else:
            return self.NotReached()

    def handle_once_timedout(self):
        PIO.set_target_twist_surge(0)
        return self.TimedOut()
    
    def timeout(self):
        return self.m_timeout

        

class OldApproachBuoyClosed(TimedState):
    class NotReached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass
    
    surge_speed: Param[float]
    heave_factor: Param[float]
    yaw_factor: Param[float]
    m_timeout: Param[float]

    def initialize(self, prev_outcome) -> None:
        super().initialize(prev_outcome)
        self.most_recent_results = prev_outcome.glyph_results
        self.glyph = prev_outcome.glyph

        self.preferred = Gbl.preferred_glyph()

        self.seen_preferred = self.glyph == self.preferred


    def handle_if_not_timedout(self):
        if self.seen_preferred:
            # TODO: check if found before setting it, otherwise we lose info
            self.most_recent_results[self.preferred] = PIO.query_glyph(self.preferred)
        else:
            self.most_recent_results.update(PIO.query_all_glyphs())
            if self.preferred in self.most_recent_results:
                self.seen_preferred = True
                self.glyph = self.preferred

        # Use PID with the heave position/percentage 
        PIO.set_target_twist_heave(
            self.most_recent_results[self.glyph].y_position * self.heave_factor
        )
        # Use setpoint for yaw angle
        PIO.set_target_twist_yaw(
            self.most_recent_results[self.glyph].x_theta * self.yaw_factor
        )
        print(self.glyph.name, self.most_recent_results[self.glyph].x_theta)  
        
        hit = PIO.buoy_collision
        
        if hit:
            if not Gbl.second_glyph:
                Gbl.second_glyph = True
                return HitBuoyFirst()
            else:
                return HitBuoySecond()
        else:
            return self.NotReached()

    def handle_once_timedout(self):
        PIO.set_target_twist_surge(0)
        return self.TimedOut()
    
    def timeout(self) -> float:
        return self.m_timeout


class FallBack(TimedState):
    class NotReached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    m_timeout: Param[float]
    speed: Param[float]

    def handle_if_not_timedout(self):
        PIO.set_target_twist_surge(-1.0 * self.speed) 
        return self.NotReached()

    def handle_once_timedout(self):
        PIO.set_target_twist_surge(0)
        return self.TimedOut() 
    
    def timeout(self) -> float:
        return self.m_timeout


class Ascend(TimedState):
    class NotReached(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass
    
    target_depth: Param[float]
    m_timeout: Param[float]
    
    def handle_if_not_timedout(self):
        PIO.set_target_pose_heave(self.target_depth)

        if PIO.Pose.heave < self.target_depth:
            return self.Reached()
        else:
            return self.NotReached()
        
    def handle_once_timedout(self):
        return self.TimedOut()
    
    def timeout(self) -> float:
        return self.m_timeout


class PassBuoy(TimedState):
    class NotReached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    m_timeout: Param[float]
    speed: Param[float]

    def handle_if_not_timedout(self):
        PIO.set_target_twist_surge(self.speed) 

        return self.NotReached()

    def handle_once_timedout(self):
        PIO.set_target_twist_surge(0)
        return self.TimedOut() 
    
    def timeout(self) -> float:
        return self.m_timeout


class FindGlyph(TimedState):
    class GlyphNotSeen(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    m_timeout: Param[float]
    speed: Param[float]

    def handle_if_not_timedout(self):
        PIO.set_target_twist_surge(-self.speed)

        res = search_for_glyph(Gbl.preferred_glyph())

        if res is not None:
            return res

        return self.GlyphNotSeen()

    def handle_once_timedout(self):
        PIO.set_target_twist_surge(0)
        return self.TimedOut() 

    def timeout(self) -> float:
        return self.m_timeout

class Pause(TimedState):
    class TimedOut(NamedTuple):
        pass

    m_timeout: Param[float]

    def handle_if_not_timedout(self):
        
        res = PIO.query_all_glyphs()

        if Glyph.taurus in res:
            return SeenGlyph(glyph_results=res[Glyph.taurus])
        # elif Glyph.auriga in res:
        else:
            return SeenGlyph(glyph_results=res[Glyph.auriga])
        
        
    def handle_once_timedout(self):
        return self.TimedOut()
    
    def timeout(self) -> float:
        return self.m_timeout

class ContingencySubmerge(State):
    class Submerged(NamedTuple):
        pass
    class Submerging(NamedTuple):
        pass

    target_depth: Param[float]
    threshold: Param[float]

    def handle(self):
        PIO.set_target_pose_heave(self.target_depth)
        
        res = PIO.query_all_glyphs()
        
        if Glyph.taurus in res:
            return SeenGlyph(glyph_results=res[Glyph.taurus])
        elif Glyph.auriga in res:
            return SeenGlyph(glyph_results=res[Glyph.auriga])
        
        if PIO.is_heave_within_threshold(threshold=self.threshold):
            return self.Submerged()

        return self.Submerging()


class ContingencyApproach(TimedState):
    class Approaching(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    surge_speed: Param[float]
    m_timeout: Param[float]

    def handle_if_not_timedout(self):
        PIO.set_target_twist_surge(self.surge_speed)
        
        hit = PIO.buoy_collision
        
        if hit:
            return HitBuoySecond()
        
        return self.Approaching()
    
    def handle_once_timedout(self):        
        PIO.set_target_twist_surge(0)
        return self.TimedOut()
    
    def timeout(self) -> float:
        return self.m_timeout
