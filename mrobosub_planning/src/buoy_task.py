from umrsm import Outcome, TimedState, State, Param
from periodic_io import PIO, Gbl, angle_error, Glyph
from mrobosub_msgs.srv import ObjectPositionResponse
from typing import Mapping

SeenGlyph = Outcome.make('SeenGlyph', glyph_results=Mapping[Glyph, ObjectPositionResponse], glyph = Glyph)
HitBuoyFirst = Outcome.make('HitBuoyFirst')
HitBuoySecond = Outcome.make('HitBuoySecond')


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
    GlyphNotSeen = Outcome.make('GlyphNotSeen')
    TimedOut = Outcome.make('TimedOut')
    
    surge_speed: Param[float]
    timeout: Param[int]

    def initialize(self, prev_outcome) -> None:
        super().initialize(prev_outcome)
        if hasattr(prev_outcome, 'angle'):
            self.target_yaw = prev_outcome.angle
        else:
            self.target_yaw = PIO.Pose.yaw

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)
        PIO.set_target_twist_surge(self.surge_speed)
        

        seen_glyph_outcome = search_for_glyph(Gbl.preferred_glyph())
        hit = PIO.buoy_collision

        if seen_glyph_outcome:
            # possible change: add some counter to increase the likelihood that we get our preferred glyph?
            return seen_glyph_outcome
        elif hit:
            return HitBuoyFirst()
        else:
            return self.GlyphNotSeen()
    
    def handle_once_timedout(self) -> None:        
        PIO.set_target_twist_surge(0)




class CenterHeaveGlyph(TimedState):
    NotCentered = Outcome.make('NotCentered')
    Centered = Outcome.make('Centered', glyph=Glyph, last_data=ObjectPositionResponse)
    TimedOut = Outcome.make('TimedOut')

    heave_down_speed: Param[float] 
    heave_up_speed: Param[float]
    deadband: Param[int] # pixels
    timeout: Param[float]


    def initialize(self, prev_outcome: SeenGlyph) -> None:
        super().initialize(prev_outcome)
        self.most_recent_results = prev_outcome.glyph_results
        self.glyph = prev_outcome.glyph
        self.glyph_y_diff = self.most_recent_results[self.glyph].y_position

    def handle_if_not_timedout(self):
        query_res = PIO.query_glyph(self.glyph)
        if query_res.found:
            self.glyph_y_diff = query_res.y_position
            self.most_recent_results[self.glyph] = query_res

        # use updated info if we have it, otherwise continue with old info? is this bad
        if abs(self.glyph_y_diff) < self.deadband:
            return self.Centered(self.glyph, self.most_recent_results[self.glyph])
        elif self.y_position > 0: # TODO: check sign?
            PIO.set_target_twist_heave(heave_down_speed)
        else:
            PIO.set_target_twist_heave(heave_up_speed)

        return self.NotCentered()



class CenterYawGlyph(TimedState):
    NotReached = Outcome.make('NotReached')
    TimedOut = Outcome.make('TimedOut')

    surge_speed: Param[float]
    yaw_factor: Param[float]
    timeout: Param[int]

    def initialize(self, prev_outcome: CenterHeaveGlyph.Centered):
        super().initialize(prev_outcome)
        self.glyph = prev_outcome.glyph
        self.angle_diff = prev_outcome.last_data.x_theta
        self.target_heave = PIO.Pose.heave

    def handle_if_not_timedout(self):
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

    def handle_once_timedout(self) -> None:
        PIO.set_target_twist_surge(0)

        

class OldApproachBuoyClosed(TimedState):
    NotReached = Outcome.make('NotReached')
    TimedOut = Outcome.make('TimedOut')
    
    surge_speed: Param[float]
    heave_factor: Param[float]
    yaw_factor: Param[float]
    timeout: Param[int]

    def initialize(self, prev_outcome) -> None:
        super().initialize(prev_outcome)
        self.most_recent_results = prev_outcome.glyph_results
        self.glyph = prev_outcome.glyph

        self.preferred = Gbl.preferred_glyph()

        self.seen_preferred = self.glyph == self.preferred


    def handle_if_not_timedout(self) -> Outcome:
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
            if not Gbl.second_glpyh:
                Gbl.second_glpyh = True
                return HitBuoyFirst()
            else:
                return HitBuoySecond()
        else:
            return self.NotReached()

    def handle_once_timedout(self) -> None:
        PIO.set_target_twist_surge(0)


class FallBack(TimedState):
    NotReached = Outcome.make('NotReached')
    TimedOut = Outcome.make('TimedOut')

    timeout: Param[int]
    speed: Param[float]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_twist_surge(-1*self.speed) 

        return self.NotReached()

    def handle_once_timedout(self) -> None:
        PIO.set_target_twist_surge(0) 


class Ascend(TimedState):
    NotReached = Outcome.make("Unreached")
    Reached = Outcome.make("Reached")
    TimedOut = Outcome.make("TimedOut")
    
    target_depth: Param[float]
    timeout: Param[int]
    
    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_heave(self.target_depth)

        if PIO.Pose.heave < self.target_depth:
            return self.Reached()
        else:
            return self.NotReached()


class PassBuoy(TimedState):
    NotReached = Outcome.make('NotReached')
    TimedOut = Outcome.make('TimedOut')

    timeout: Param[int]
    speed: Param[float]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_twist_surge(self.speed) 

        return self.NotReached()

    def handle_once_timedout(self) -> None:
        PIO.set_target_twist_surge(0) 


class FindGlyph(TimedState):
    GlyphNotSeen = Outcome.make('GlyphNotSeen')
    TimedOut = Outcome.make('TimedOut')

    timeout: Param[int]
    speed: Param[float]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_twist_surge(-self.speed) 

        res = PIO.query_all_glyphs()

        if len(res):
            return SeenGlyph(glyph_results=res)
        
        return self.GlyphNotSeen()

    def handle_once_timedout(self) -> None:
        PIO.set_target_twist_surge(0) 


class Pause(TimedState):
    TimedOut = Outcome.make('TimedOut')

    timeout: Param[float]

    def handle_if_not_timedout(self) -> Outcome:
        
        res = PIO.query_all_glyphs()

        if Glyph.taurus in res:
            return SeenGlyph(glyph_results=res[Glyph.taurus])
        elif Glyph.auriga in res:
            return SeenGlyph(glyph_results=res[Glyph.auriga])
    
            
class ContingencySubmerge(State):
    Submerged = Outcome.make('Submerged')
    Submerging = Outcome.make('Submerging')

    target_depth: Param[float]
    threshold: Param[float]

    def handle(self) -> Outcome:
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
    Approaching = Outcome.make('Approaching')
    TimedOut = Outcome.make('TimedOut')

    surge_speed: Param[float]
    timeout: Param[int]

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_twist_surge(self.surge_speed)
        
        hit = PIO.buoy_collision
        
        if hit:
            return HitBuoySecond()
        
        return self.Approaching()
    
    def handle_once_timedout(self) -> None:        
        PIO.set_target_twist_surge(0)
