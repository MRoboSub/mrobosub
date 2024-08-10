from abstract_states import TimedState, TurnToYaw, AlignPathmarker
from periodic_io import PIO, ImageTarget
from mrobosub_msgs.srv import ObjectPositionResponse  # type: ignore
import rospy
from typing import NamedTuple, Union, Optional, List
from circumnavigate_states import CircumnavigateOpenDiscreteDiamondTurns


class SeenGateImageType(NamedTuple):
    position: ObjectPositionResponse
    image_seen: ImageTarget


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
        PIO.set_target_pose_heave(0.75)

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

    timeout: float = 150.0
    surge_speed: float = 0.15
    found_image_threshold = 50

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        PIO.activate_zed()
        self.times_seen = 0

    def handle_if_not_timedout(self) -> Union[SeenGateImage, None]:
        PIO.set_target_twist_surge(self.surge_speed)
        PIO.set_target_pose_heave(0.75)

        # blue_response = PIO.query_image(ImageTarget.GATE_BLUE)
        red_response = PIO.query_image(ImageTarget.GATE_RED)
        res_exists = red_response.found # or blue_response.found

        if not res_exists:
            return None

        if self.times_seen >= self.found_image_threshold:
            # if red_response.found:
                # Prefer red because red bin is probably easier to see
                return self.SeenGateImage(red_response, ImageTarget.GATE_RED)
            # else:
            #     return self.SeenGateImage(blue_response, ImageTarget.GATE_BLUE)


        self.times_seen += 1
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)

        return self.TimedOut()


class ApproachGateImage(TimedState):
    class GoneThroughGate(NamedTuple):
        planet: ImageTarget

    class FoundBuoyPathmarker(NamedTuple):
        angle: float

    class TimedOut(NamedTuple):
        pass

    timeout: float = 25.0
    lost_image_threshold: int = 50
    yaw_threshold: float = 2.0

    def __init__(self, prev_outcome: NamedTuple) -> None:
        super().__init__(prev_outcome)
        if not isinstance(prev_outcome, SeenGateImageType):
            raise TypeError(f"Expected type SeenGateImageType, received {prev_outcome}")
        self.image_seen = prev_outcome.image_seen
        self.last_target_yaw = PIO.Pose.yaw + prev_outcome.position.x_theta
        self.times_not_seen = 0
        PIO.activate_zed()

    def handle_if_not_timedout(self) -> Union[GoneThroughGate, None]:
        # Precondition: you have already seen a glyph. You are trying to update
        if self.image_seen is None:
            raise ValueError(f"Expected to have seen a planet by now")

        resp = PIO.query_image(self.image_seen)
        resp_blue = PIO.query_image(ImageTarget.GATE_BLUE)
        if not resp.found and not resp_blue.found:
            self.times_not_seen += 1
            if self.times_not_seen >= self.lost_image_threshold:
                return self.GoneThroughGate(planet=self.image_seen)
        else:
            self.times_not_seen = 0
            self.last_target_yaw = PIO.Pose.yaw + resp.x_theta * 0.05

        # pm_angle = self.query_pathmarker()
        # if pm_angle is not None:
        #    return self.FoundBuoyPathmarker(angle=pm_angle)

        PIO.set_target_pose_yaw(self.last_target_yaw)
        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class AlignBuoyPathmarker(AlignPathmarker):
    class AlignedToBuoy(NamedTuple):
        pass
    class NoMeasurements(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    yaw_threshold = 2.5
    timeout = 10.

    def handle_if_not_timedout(self) -> Union[NamedTuple, None]:
        outcome = super().handle_if_not_timedout()
        if self.iter == 100 and hasattr(self, 'target_angle'):
            self.target_angle %= 360
            self.target_angle += 360
            self.target_angle %= 360
            if 90 <= self.target_angle < 180:
                self.target_angle += 180
            if 180 <= self.target_angle < 270:
                self.target_angle -= 180
            print(f'adjusted_setpoint: {self.target_angle=}')
        return outcome

    def handle_aligned(self) -> AlignedToBuoy:
        return self.AlignedToBuoy()

    def handle_no_measurements(self) -> NoMeasurements:
        return self.NoMeasurements()

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()


class GuessBuoyAngle(TurnToYaw):
    class Reached(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    target_yaw = -10.
    yaw_threshold = 2.0
    settle_time = 1.0
    timeout = 10.0

    def handle_reached(self):
        return self.Reached()
    def handle_once_timedout(self):
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
    timeout: float = 15.0

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        self.timer = rospy.get_time()

    def handle_if_not_timedout(self) -> Union[Reached, None]:
        PIO.set_target_twist_surge(0.0)
        PIO.set_target_pose_heave(0.4)

        target_yaw = 0
        PIO.set_target_pose_yaw(target_yaw)

        if not PIO.is_yaw_within_threshold(self.yaw_threshold):
            self.timer = rospy.get_time()

        if rospy.get_time() - self.timer >= 1:
            return self.Reached(target_yaw)

        return None

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()
