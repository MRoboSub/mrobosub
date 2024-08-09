import rospy
from umrsm import State
from abstract_states import AlignPathmarker, TimedState
from periodic_io import PIO, ImageDetections, ImageTarget
from mrobosub_msgs.srv import ObjectPositionResponse  # type: ignore
from typing import Dict, Optional, Tuple, Union, NamedTuple


class SeenBuoyType(NamedTuple):
    buoy_results: ObjectPositionResponse


class ApproachBuoyOpen(TimedState):
    class SeenBuoy(SeenBuoyType):
        pass

    class TimedOut(NamedTuple):
        pass

    surge_speed: float = 0.2
    timeout: float = 20

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        PIO.activate_zed()
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
        PIO.activate_zed()
        if not isinstance(prev_outcome, SeenBuoyType):
            raise TypeError(f"Expected a SeenBuoyType outcome, received {prev_outcome}")
        self.most_recent_results = prev_outcome.buoy_results
        self.buoy_y_theta = self.most_recent_results.y_theta

    def handle_if_not_timedout(self) -> Union[Centered, None]:
        query_res = PIO.query_buoy()
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

    radius_thold: float = 25.
    unseen_thold: float = 20.0
    surge_speed: float = 0.2
    yaw_factor: float = 0.5
    timeout: float = 40.0

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        PIO.activate_zed()
        if type(prev_outcome) != CenterHeaveBuoy.Centered:
            raise TypeError(type(prev_outcome))
        self.bbox_area = (
            prev_outcome.last_data.confidence
        )  ##Need to replace with actual bbox area
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

        if query_res.found:
            PIO.set_target_pose_yaw(PIO.Pose.yaw + self.angle_diff * self.yaw_factor)

        if self.bbox_area >= self.radius_thold:
            return self.CloseToBuoy()

        if rospy.get_time() - self.unseen_time >= self.unseen_thold:
            print("Unseen for longer than time threshold")
            # Assume we cant see it because we're too close or past the buoy
            return self.CloseToBuoy()

        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_yaw(0)
        return self.TimedOut()


class AlignBinsPathmarker(AlignPathmarker):
    class AlignedToBins(NamedTuple):
        pass
    class NoMeasurements(NamedTuple):
        pass
    class TimedOut(NamedTuple):
        pass

    yaw_threshold = 2.
    timeout = 10.

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        self.iter = -50

    def handle_if_not_timedout(self) -> Union[NamedTuple, None]:
        if self.iter < 0:
            PIO.set_target_twist_surge(-0.1)
            PIO.set_target_twist_sway(-0.1)
            return None
        outcome = super().handle_if_not_timedout()
        if self.iter == 100 and hasattr(self, 'target_angle'):
            if PIO.query_buoy().found:
                self.target_angle -= 180
        return outcome

    def handle_aligned(self) -> AlignedToBins:
        return self.AlignedToBins()

    def handle_no_measurements(self) -> NoMeasurements:
        return self.NoMeasurements()

    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()

