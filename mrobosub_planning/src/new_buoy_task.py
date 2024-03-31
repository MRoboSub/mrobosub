import rospy
from umrsm import State
from abstract_states import TimedState
from periodic_io import PIO
from mrobosub_msgs.srv import ObjectPositionResponse # type: ignore
from typing import Dict, Optional, Tuple, Union, NamedTuple


class ApproachBuoyOpen(TimedState):
    class ArrivedAtBuoy(NamedTuple):
        pass
    class LostBuoy(NamedTuple):
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

        seen_glyph = search_for_glyph(Gbl.preferred_glyph())
        if seen_glyph is not None:
            print(seen_glyph[1])
        # hit = PIO.buoy_collision

        if seen_glyph is not None:
            # possible change: add some counter to increase the likelihood that we get our preferred glyph?
            return self.SeenGlyph(*seen_glyph)
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
    deadband: int = 50 # pixels, or maybe 5 degrees
    timeout: float = 40.0

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        if not isinstance(prev_outcome, SeenGlyphType):
            raise TypeError(
                f"Expected a SeenGlyph-like outcome, received {prev_outcome}"
            )
        self.most_recent_results = prev_outcome.glyph_results
        self.glyph = prev_outcome.glyph
        self.glyph_y_theta = self.most_recent_results[self.glyph].y_theta
        self.preferred = Gbl.preferred_glyph()


    def handle_if_not_timedout(self) -> Union[Centered, None]:
        query_res = PIO.query_glyph(self.glyph)
        if query_res.found:
            self.glyph_y_theta = query_res.y_theta
            self.most_recent_results[self.glyph] = query_res

        # use updated info if we have it, otherwise continue with old info? is this bad
        if abs(self.glyph_y_theta) < self.deadband:
            return self.Centered(self.glyph, self.most_recent_results[self.glyph])
        elif self.glyph_y_theta > 0:
            PIO.set_target_twist_heave(0.17)
        else:
            PIO.set_target_twist_heave(0.0)
        PIO.set_target_twist_surge(0)

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

    bbox_area_thold: float = 0.05
    unseen_thold: float = 10.
    surge_speed: float = 0.2
    yaw_factor: float = 0.0005
    timeout: float = 40.0

    def __init__(self, prev_outcome):
        super().__init__(prev_outcome)
        if type(prev_outcome) != CenterHeaveGlyph.Centered:
            raise TypeError(type(prev_outcome))
        self.hit_time_thold = 2
        self.bbox_area = prev_outcome.last_data.x_position
        self.unseen_time = rospy.get_time()

        self.glyph = prev_outcome.glyph
        self.angle_diff = prev_outcome.last_data.x_theta
        self.target_heave = PIO.Pose.heave

        self.hit_start_time = None

    def handle_if_not_timedout(self) -> Union[HitBuoyFirst, HitBuoySecond, None]:
        print(self.glyph)
        query_res = PIO.query_glyph(self.glyph)
        if query_res.found:
            self.angle_diff = query_res.x_theta
            self.bbox_area = query_res.x_position
            self.unseen_time = rospy.get_time()

        PIO.set_target_pose_heave(self.target_heave)
        PIO.set_target_twist_surge(self.surge_speed)

        # Use setpoint for yaw angle
        if query_res.found:
            PIO.set_target_pose_yaw(PIO.Pose.yaw + self.angle_diff/2 - 10.0)
            # print(PIO.Pose.yaw + self.angle_diff/2)
        
        # hit = PIO.buoy_collision and \
        #     (self.bbox_area >= self.bbox_area_thold or self.frames_unseen >= self.unseen_thold)
        
        if (
            # (self.bbox_area >= self.bbox_area_thold or 
            rospy.get_time() - self.unseen_time >= self.unseen_thold
            and self.hit_start_time is None
        ):
            if(self.bbox_area >= self.bbox_area_thold): print(f"BBOX AT {self.bbox_area}")
            if(rospy.get_time() - self.unseen_time >= self.unseen_thold): print(f"UNSEEN FOR {self.unseen_thold}")
            self.hit_start_time = rospy.get_time()
            
        if self.hit_start_time is not None and rospy.get_time() - self.hit_start_time >= self.hit_time_thold:
            if not Gbl.second_glyph:
                Gbl.second_glyph = True
                Gbl.first_hit_glyph = self.glyph
                return self.HitBuoyFirst()
            else:
                return self.HitBuoySecond()
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_yaw(0)
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
        PIO.set_target_twist_surge(-self.speed)
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
    speed: float = 0.15

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

    class TimedOut(NamedTuple):
        pass

    timeout: float = 3.0

    def handle_if_not_timedout(self) -> Union[SeenGlyph, None]:
        res = PIO.query_all_glyphs()

        if Glyph.TAURUS in res:
            return self.SeenGlyph(res, Glyph.TAURUS)
        elif Glyph.AURIGA:
            return self.SeenGlyph(res, Glyph.AURIGA)
        return None

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
