from abstract_states import TimedState
from periodic_io import PIO, Glyph, Gbl
from mrobosub_msgs.srv import ObjectPositionResponse  # type: ignore
import rospy
from typing import NamedTuple, Union


class SeenGateImageType(NamedTuple):
    position: ObjectPositionResponse
    glyph_seen: Glyph


class AlignGate(TimedState):
    class ReachedAngle(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    target_yaw: float = 0.0
    timeout: float = 10.0
    yaw_threshold: float = 2.0

    def handle_if_not_timedout(self) -> Union[ReachedAngle, None]:
        PIO.set_target_pose_yaw(self.target_yaw)

        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.ReachedAngle()
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_pose_yaw(0)
        return self.TimedOut()


class ApproachGate(TimedState):
    class SeenGateImage(SeenGateImageType):
        pass

    class TimedOut(NamedTuple):
        pass

    timeout: float = 26.0
    surge_speed: float = 0.2
    found_image_threshold = 50

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        self.times_seen = 0

    def handle_if_not_timedout(self) -> Union[SeenGateImage, None]:
        PIO.set_target_twist_surge(self.surge_speed)

        abydos_response = PIO.query_glyph(Glyph.ABYDOS)
        earth_response = PIO.query_glyph(Glyph.EARTH)
        res_exists = abydos_response.found or earth_response.found

        if not res_exists:
            return None

        if self.times_seen >= self.found_image_threshold:
            if abydos_response.found:
                return self.SeenGateImage(abydos_response, Glyph.ABYDOS)
            else:
                return self.SeenGateImage(earth_response, Glyph.EARTH)

        self.times_seen += 1
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)

        return self.TimedOut()


class ApproachGateImage(TimedState):
    class GoneThroughGate(NamedTuple):
        planet: Glyph

    class FoundBuoyPathMarker(NamedTuple):
        angle: float

    class TimedOut(NamedTuple):
        pass

    timeout: float = 25.0
    lost_image_threshold: int = 300
    yaw_threshold: float = 2.0

    def __init__(self, prev_outcome: NamedTuple) -> None:
        super().__init__(prev_outcome)
        if not isinstance(prev_outcome, SeenGateImageType):
            raise TypeError(f"Expected type SeenGateImageType, received {prev_outcome}")
        Gbl.planet_seen = prev_outcome.glyph_seen
        self.last_target_yaw = PIO.Pose.yaw + prev_outcome.position.x_theta
        self.times_not_seen = 0

    def handle_if_not_timedout(self) -> Union[GoneThroughGate, None]:
        # Precondition: you have already seen a glyph. You are trying to update
        if Gbl.planet_seen is None:
            raise ValueError(f"Expected to have seen a planet by now")

        resp = PIO.query_glyph(Gbl.planet_seen)
        if not resp.found:
            self.times_not_seen += 1
            if self.times_not_seen >= self.lost_image_threshold:
                return self.GoneThroughGate(planet=Gbl.planet_seen)
        else:
            self.times_not_seen = 0
            self.last_target_yaw = PIO.Pose.yaw + resp.x_theta * 0.5

        # pm_angle = self.query_pathmarker()
        # if pm_angle is not None:
        #    return self.FoundBuoyPathMarker(angle=pm_angle)

        PIO.set_target_pose_yaw(self.last_target_yaw)
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class AlignPathmarker(TimedState):
    # class SeenGlyph(SeenGlyphType):
    #     pass
    class Aligned(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    yaw_threshold: float = 2.0
    timeout: int = 10

    def __init__(self, prev_outcome: NamedTuple) -> None:
        super().__init__(prev_outcome)
        self.last_known_angle = None
        if isinstance(prev_outcome, ApproachGateImage.FoundBuoyPathMarker):
            self.last_known_angle = prev_outcome.angle
        else:
            print(f"Expected type FoundBuoyPathMarker, received {prev_outcome}") 
        


    def handle_if_not_timedout(self) -> Union[Aligned, TimedOut, None]:
        pm_resp = PIO.query_pathmarker()
        print(pm_resp)
        if pm_resp is not None:
            self.last_known_angle = pm_resp

        if self.last_known_angle != None:
            PIO.set_target_pose_yaw(self.last_known_angle)
        else:
            return None

        if PIO.is_yaw_within_threshold(self.yaw_threshold):
            return self.Aligned()
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class Spin(TimedState):
    timeout: float = 30.0

    class TimedOut(NamedTuple):
        pass

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        self.timeout = 15.0

    def handle_if_not_timedout(self) -> None:
        PIO.set_target_twist_yaw(0.12)
        PIO.set_target_pose_heave(1)
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_yaw(0)
        return self.TimedOut()


class SpinFinish(TimedState):
    class Reached(NamedTuple):
        angle: float

    class TimedOut(NamedTuple):
        pass

    yaw_threshold: float = 2.0
    timeout: float = 30.0

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        self.timer = rospy.get_time()

    def handle_if_not_timedout(self) -> Union[Reached, None]:
        target_yaw = 0
        PIO.set_target_pose_yaw(target_yaw)

        if not PIO.is_yaw_within_threshold(self.yaw_threshold):
            self.timer = rospy.get_time()

        if rospy.get_time() - self.timer >= 1:
            return self.Reached(target_yaw)

        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()