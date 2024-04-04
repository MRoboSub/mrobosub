import rospy
from umrsm import State
from abstract_states import TimedState
from periodic_io import PIO, Gbl, GlyphDetections, Glyph
from mrobosub_msgs.srv import ObjectPositionResponse # type: ignore
from typing import Dict, Optional, Tuple, Union, NamedTuple

class SeenBuoyType():
    buoy_results: ObjectPositionResponse
    def __init__(self, buoy_res: ObjectPositionResponse):
        self.buoy_results = buoy_res



class ApproachBuoyOpen(TimedState):
    class SeenBuoy(SeenBuoyType):
        pass

    class TimedOut(NamedTuple):
        pass

    surge_speed: float = 0.2
    timeout: float = 20

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        self.target_yaw = getattr(prev_outcome, "angle", PIO.Pose.yaw)

    def handle_if_not_timedout(self) -> Union[SeenBuoy, None]:
        PIO.set_target_pose_yaw(self.target_yaw)
        PIO.set_target_twist_surge(self.surge_speed)

        buoyPosition = PIO.query_buoy()
        if buoyPosition.found:
            return self.SeenBuoy(buoyPosition)
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        return self.TimedOut()


class CenterHeaveBuoy(TimedState):
    class Centered(NamedTuple):
        last_data: ObjectPositionResponse

    class TimedOut(NamedTuple):
        last_data: ObjectPositionResponse

    heave_down_speed: float = 0.2
    heave_up_speed: float = -0.1
    deadband: int = 5 # pixels, or maybe 5 degrees
    timeout: float = 40.0

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        if not isinstance(prev_outcome, SeenBuoyType):
            raise TypeError(
                f"Expected a SeenBuoyType outcome, received {prev_outcome}"
            )
        self.most_recent_results = prev_outcome.buoy_results
        self.buoy_y_theta = self.most_recent_results.y_theta


    def handle_if_not_timedout(self) -> Union[Centered, None]:
        query_res: ObjectPositionResponse = PIO.query_buoy()
        if query_res.found:
            self.buoy_y_theta = query_res.y_theta

        # use updated info if we have it, otherwise continue with old info? is this bad
        if abs(self.buoy_y_theta) < self.deadband:
            return self.Centered(self.most_recent_results)
        elif self.buoy_y_theta > 0:
            PIO.set_target_twist_heave(self.heave_down_speed)
        else:
            PIO.set_target_twist_heave(self.heave_up_speed)
        PIO.set_target_twist_surge(0)

        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut(self.most_recent_results)


class CenterYawBuoy(TimedState):
    class CloseToBuoy(NamedTuple):
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
        if type(prev_outcome) != CenterHeaveBuoy.Centered:
            raise TypeError(type(prev_outcome))
        self.bbox_area = prev_outcome.last_data.x_position ##Need to replace with actual bbox area
        self.unseen_time = rospy.get_time()

        self.buoy_position: ObjectPositionResponse = prev_outcome.last_data
        self.angle_diff = prev_outcome.last_data.x_theta
        self.target_heave = PIO.Pose.heave


    def handle_if_not_timedout(self) -> Union[CloseToBuoy, None]:
        query_res: ObjectPositionResponse = PIO.query_buoy()
        if query_res.found:
            self.angle_diff = query_res.x_theta
            self.bbox_area = query_res.x_position
            self.unseen_time = rospy.get_time()

        PIO.set_target_pose_heave(self.target_heave)
        PIO.set_target_twist_surge(self.surge_speed)

        # Use setpoint for yaw angle
        if query_res.found:
            PIO.set_target_pose_yaw(PIO.Pose.yaw + self.angle_diff/2)
            # print(PIO.Pose.yaw + self.angle_diff/2)
        

        if(self.bbox_area >= self.bbox_area_thold):
            print("Made it to buoy!")
            return self.CloseToBuoy()
        
        if(rospy.get_time() - self.unseen_time >= self.unseen_thold):
            print("Unseen for longer than time threshold")
            #TODO: Could add logic for if the sub loses the buoy, maybe have it spin and try to find the buoy 

        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_yaw(0)
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
    


#####
#OLD BUOY task stuff just for compilation
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

