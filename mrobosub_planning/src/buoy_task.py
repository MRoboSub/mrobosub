from umrsm import Outcome, TimedState, State, Param
from periodic_io import PIO, Gbl, angle_error, Glyph

SeenGlyph = Outcome.make('SeenGlyph', glyph_results=Mapping[Glyph, GlyphPositionResponse], glyph = Glyph)
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
    if Gbl.planet_seen == Glyph.abydos_poo:
        return Glyph.taurus
    else:
        return Glyph.auriga


class ApproachBuoyOpen(TimedState):
    GlyphNotSeen = Outcome.make('GlyphNotSeen')
    TimedOut = Outcome.make('TimedOut')
    
    surge_speed: Param[float]
    timeout: Param[int]

    def initialize(self, prev_outcome: Aligned) -> None:
        super().initialize()
        self.target_yaw = prev_outcome.angle

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)
        PIO.set_target_twist_surge(self.surge_speed)
        
        seen_glyph_outcome = search_for_glyph(first_glyph())
        hit = PIO.buoy_collision

        if seen_glyph_outcome:
            return seen_glyph_outcome
        elif hit:
            return HitBuoyFirst()
        else:
            return self.GlyphNotSeen()
    
    def handle_once_timedout(self) -> None:        
        PIO.set_target_twist_surge(0)

        
class ApproachBuoyClosed(TimedState):
    NotReached = Outcome.make('NotReached')
    
    surge_speed: Param[float]
    heave_factor: Param[float]
    yaw_factor: Param[float]
    timeout: Param[int]

    def initialize(self, prev_outcome: SeenBuoy) -> None:
        super().initialize()
        self.most_recent_results = prev_outcome.glyph_results
        self.glyph = prev_outcome.glyph

        if not Gbl.second_glpyh:
            if Gbl.planet_seen == Glyph.abydos_poo:
                self.preferred = Glyph.taurus
            else:
                self.preferred = Glyph.auriga
        else:
            if Gbl.planet_seen == Glyph.abydos_poo:
                self.preferred = Glyph.serpens_caput
            else:
                self.preferred = Glyphh.cetus

        self.seen_preferred = self.glyph == self.preferred


    def handle_if_not_timedout(self) -> Outcome:
        if self.seen_preferred:
            self.most_recent_results.update(PIO.query_glyph(self.preferred))
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
        
        hit = PIO.buoy_collision
        
        if hit:
            return HitBuoySecond() if Gbl.second_glpyh else HitBuoyFirst()
        
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
    Submerged = Outcome.make("Submerged")
    TimedOut = Outcome.make("TimedOut")
    
    target_depth: Param[float]
    timeout: Param[int]
    
    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_heave(self.target_depth)

        if PIO.Pose.heave < self.target_depth:
            return self.Submerged()
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
