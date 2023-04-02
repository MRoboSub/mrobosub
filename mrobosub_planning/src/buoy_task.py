from umrsm import Outcome, TimedState, State, Param
from periodic_io import PIO, Gbl, angle_error

class ApproachBuoyOpen(TimedState):
    GlyphNotSeen = Outcome.make('GlyphNotSeen')
    SeenGlyph = Outcome.make('SeenGlyph', glyph_results=Mapping[Glyph, GlyphPositionResponse])
    HitBuoy = Outcome.make('HitBuoy')
    TimedOut = Outcome.make('TimedOut')
    
    surge_speed: Param[float]
    timeout: Param[int]

    def initialize(self, prev_outcome: Aligned) -> None:
        super().initialize()
        self.target_yaw = prev_outcome.angle

    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)
        PIO.set_target_twist_surge(self.surge_speed)
        
        res = PIO.query_all_glyphs()
        hit = Gbl.buoy_collision
        
        if len(res):
            return self.SeenBuoy(glyph_results=res)
        
        if hit:
            return self.HitBuoy()
        
        return self.BuoyNotSeen()
    
    def handle_once_timedout(self) -> None:        
        PIO.set_target_twist_surge(0)

        
class ApproachBuoyClosed(TimedState):
    NotReached = Outcome.make('NotReached')
    HitBuoyFirst = Outcome.make('HitBuoyFirst')
    HitBuoySecond = Outcome.make('HitBuoySecond')
    
    surge_speed: Param[float]
    timeout: Param[int]

    def initialize(self, prev_outcome: SeenBuoy) -> None:
        super().initialize()
        self.most_recent_results = prev_outcome.glyph_results

    def handle_if_not_timedout(self) -> Outcome:
        self.most_recent_results |= PIO.query_all_glyphs()

        # Use PID with the heave position/percentage 
        # Use setpoint for yaw angle
        
        hit = Gbl.buoy_collision
        
        if hit:
            return self.HitBuoySecond() if Gbl.second_glpyh else self.HitBuoyFirst()
        
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
