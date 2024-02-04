from umrsm import State
from abstract_states import TimedState
from periodic_io import PIO, Gbl, GlyphDetections, Glyph
from mrobosub_msgs.srv import ObjectPositionResponse # type: ignore
from typing import Dict, Optional, Tuple, Union, NamedTuple


class SeenGlyphType(NamedTuple):
    glyph_results: Dict[Glyph, ObjectPositionResponse]
    glyph: Glyph


def search_for_glyph(preference) -> Optional[Tuple[GlyphDetections, Glyph]]:
    res = PIO.query_all_glyphs()
    for g in [preference, Glyph.TAURUS, Glyph.SERPENS_CAPUT, Glyph.ABYDOS, Glyph.CETUS]:
        if g in res:
            return res, g
    return None


def first_glyph():
    if Gbl.planet_seen == Glyph.ABYDOS:
        return Glyph.TAURUS
    else:
        return Glyph.AURIGA


class ApproachBuoyOpen(TimedState):
    class SeenGlyph(SeenGlyphType):
        pass

    class TimedOut(NamedTuple):
        pass

    surge_speed: float = 0.2
    timeout: float = 20

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        self.target_yaw = getattr(prev_outcome, "angle", PIO.Pose.yaw)

    def handle_if_not_timedout(self) -> Union[SeenGlyph, None]:
        PIO.set_target_pose_yaw(self.target_yaw)
        PIO.set_target_twist_surge(self.surge_speed)

        seen_glyph_outcome = search_for_glyph(Gbl.preferred_glyph())
        # hit = PIO.buoy_collision

        if seen_glyph_outcome is not None:
            # possible change: add some counter to increase the likelihood that we get our preferred glyph?
            return self.SeenGlyph(*seen_glyph_outcome)
        # elif hit:
        #     return HitBuoyFirst()
        else:
            return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        return self.TimedOut()


class CenterHeaveGlyph(TimedState):
    class Centered(NamedTuple):
        glyph: Glyph
        last_data: ObjectPositionResponse

    class TimedOut(NamedTuple):
        glyph: Glyph
        last_data: ObjectPositionResponse

    heave_down_speed: float = 0.2
    heave_up_speed: float = 0.0
    deadband: int = 50  # pixels
    timeout: float = 40.0

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        if not isinstance(prev_outcome, SeenGlyphType):
            raise TypeError(
                f"Expected a SeenGlyph-like outcome, received {prev_outcome}"
            )
        self.most_recent_results = prev_outcome.glyph_results
        self.glyph = prev_outcome.glyph
        self.glyph_y_diff = self.most_recent_results[self.glyph].y_position

    def handle_if_not_timedout(self) -> Union[Centered, None]:
        query_res = PIO.query_glyph(self.glyph)
        if query_res.found:
            self.glyph_y_diff = query_res.y_position
            self.most_recent_results[self.glyph] = query_res

        # use updated info if we have it, otherwise continue with old info? is this bad
        if abs(self.glyph_y_diff) < self.deadband:
            return self.Centered(self.glyph, self.most_recent_results[self.glyph])
        elif self.glyph_y_diff > 0:  # TODO: check sign?
            PIO.set_target_twist_heave(0.17)
        else:
            PIO.set_target_twist_heave(0)

        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut(self.glyph, self.most_recent_results[self.glyph])


class CenterYawGlyph(TimedState):
    class HitBuoyFirst(NamedTuple):
        pass

    class HitBuoySecond(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    surge_speed: float = 0.2
    yaw_factor: float = 0.004
    timeout: float = 10.0

    def __init__(self, prev_outcome):
        if type(prev_outcome) != CenterHeaveGlyph.Centered:
            raise TypeError(type(prev_outcome))
        self.surge_speed = 0.2
        self.yaw_factor = 0.004
        self.timeout = 40
        super().__init__(prev_outcome)
        self.glyph = prev_outcome.glyph
        self.angle_diff = prev_outcome.last_data.x_theta
        self.target_heave = PIO.Pose.heave

    def handle_if_not_timedout(self) -> Union[HitBuoyFirst, HitBuoySecond, None]:
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
            if not Gbl.second_glyph:
                Gbl.second_glyph = True
                return self.HitBuoyFirst()
            else:
                return self.HitBuoySecond()
        return None

    def handle_once_timedout(self):
        PIO.set_target_twist_surge(0)
        return self.TimedOut()


class OldApproachBuoyClosed(TimedState):
    class HitBuoyFirst(NamedTuple):
        pass

    class HitBuoySecond(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    surge_speed: float = 0.2
    heave_factor: float = 0.002
    yaw_factor: float = 0.004
    timeout: float = 10.0

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        self.most_recent_results = getattr(prev_outcome, "glyph_results")
        self.glyph = getattr(prev_outcome, "glyph")

        self.preferred = Gbl.preferred_glyph()

        self.seen_preferred = self.glyph == self.preferred

    def handle_if_not_timedout(self) -> Union[HitBuoyFirst, HitBuoySecond, None]:
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
                return self.HitBuoyFirst()
            else:
                return self.HitBuoySecond()
        else:
            return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        return self.TimedOut()


class FallBack(TimedState):
    class TimedOut(NamedTuple):
        pass

    timeout: float = 3.0
    speed: float = 0.2

    def handle_if_not_timedout(self) -> None:
        PIO.set_target_twist_surge(-1.0 * self.speed)
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        return self.TimedOut()


class Ascend(TimedState):
    class Reached(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    target_depth: float = 0.25
    timeout: float = 3.0

    def handle_if_not_timedout(self) -> Union[Reached, None]:
        PIO.set_target_pose_heave(self.target_depth)

        if PIO.Pose.heave < self.target_depth:
            return self.Reached()
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class PassBuoy(TimedState):
    class TimedOut(NamedTuple):
        pass

    timeout: float = 5.0
    speed: float = 0.2

    def handle_if_not_timedout(self) -> None:
        PIO.set_target_twist_surge(self.speed)
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        return self.TimedOut()


class FindGlyph(TimedState):
    class SeenGlyph(SeenGlyphType):
        pass

    class TimedOut(NamedTuple):
        pass

    timeout: float = 7.0
    speed: float = 0.2

    def handle_if_not_timedout(self) -> Union[SeenGlyph, None]:
        PIO.set_target_twist_surge(-self.speed)

        res = search_for_glyph(Gbl.preferred_glyph())

        if res is not None:
            return self.SeenGlyph(*res)
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        return self.TimedOut()


class Pause(TimedState):
    class SeenGlyph(SeenGlyphType):
        pass

    # class GlyphNotSeen(NamedTuple): # TODO
    #     pass

    class TimedOut(NamedTuple):
        pass

    timeout: float = 3.0

    def handle_if_not_timedout(self) -> SeenGlyph:

        res = PIO.query_all_glyphs()

        if Glyph.TAURUS in res:
            return self.SeenGlyph(res, Glyph.TAURUS)
        else:
            return self.SeenGlyph(res, Glyph.AURIGA)

    def handle_once_timedout(self):
        return self.TimedOut()


class ContingencySubmerge(State):
    class SeenGlyph(SeenGlyphType):
        pass

    class Submerged(NamedTuple):
        pass

    target_depth: float = 15.0
    threshold: float = 2.0

    def handle(self) -> Union[SeenGlyph, Submerged, None]:
        PIO.set_target_pose_heave(self.target_depth)

        res = PIO.query_all_glyphs()

        if Glyph.TAURUS in res:
            return self.SeenGlyph(res, Glyph.TAURUS)
        elif Glyph.AURIGA in res:
            return self.SeenGlyph(res, Glyph.AURIGA)

        if PIO.is_heave_within_threshold(threshold=self.threshold):
            return self.Submerged()
        return None


class ContingencyApproach(TimedState):
    class HitBuoySecond(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    surge_speed: float = 0.2
    timeout: float = 15.0

    def handle_if_not_timedout(self) -> Union[HitBuoySecond, None]:
        PIO.set_target_twist_surge(self.surge_speed)

        hit = PIO.buoy_collision

        if hit:
            return self.HitBuoySecond()
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        return self.TimedOut()
