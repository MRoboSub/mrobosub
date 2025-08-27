#!/usr/bin/env python
from common_states import Start, Submerge, Surface
from gate_states import AlignSlalomPathmarker, AlignGate, AlignSlalom, ApproachGate2, CenterSlalomPathmarker1, CenterSlalomPathmarker2, Spin, SpinFinish
from buoy_states import AlignBinsPathmarker, Slalom, CenterBinsPathmarker
from abstract_states import TimedState
from periodic_io import PIO
from umrsm import Outcome, TransitionMap

slalom_heave = 1.0
CustomAlignSlalomPathmarker = AlignSlalomPathmarker.with_params(angle_offset=0.0, target_heave=slalom_heave)
CustomSlalom = Slalom.with_params(target_heave=slalom_heave, surge_speed=1.0, timeout=45.) # 33

class GoToOctagon(TimedState):
    class Finished(Outcome):
        pass

    timeout: float = 40.0

    def handle_if_not_timedout(self) -> None:
        PIO.set_target_twist_surge(1.5)
        return None

    def handle_once_timedout(self) -> Outcome:
        return self.Finished()


class Forward(TimedState):
    class Finished(Outcome):
        pass

    timeout = 200.

    def handle_if_not_timedout(self) -> None:
        PIO.set_target_twist_surge(1.0)
        PIO.set_target_pose_heave(1.2)
        return None

    def handle_once_timedout(self) -> Finished:
        return self.Finished()

transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=1.),

    Submerge.Submerged: Forward,
    Forward.Finished: Surface,

    Surface.Surfaced: Surface,
    # Submerge.TimedOut: AlignGate,

    # AlignGate.ReachedAngle: ApproachGate2.with_params(timeout=30.), # 23
    # AlignGate.TimedOut: ApproachGate2.with_params(timeout=30.), # 23

    # ApproachGate2.SeenPathmarker: CenterSlalomPathmarker1,
    # ApproachGate2.TimedOut: Surface,

    # CenterSlalomPathmarker1.Centered: Spin.with_params(timeout=35.0, speed=0.5),
    # CenterSlalomPathmarker1.TimedOut: Spin.with_params(timeout=35.0, speed=0.5),

    # Spin.TimedOut: SpinFinish,

    # SpinFinish.Reached: CenterSlalomPathmarker2,
    # SpinFinish.TimedOut: CenterSlalomPathmarker2,

    # CenterSlalomPathmarker2.Centered: CustomAlignSlalomPathmarker,
    # CenterSlalomPathmarker2.TimedOut: CustomAlignSlalomPathmarker,

    # CustomAlignSlalomPathmarker.AlignedToSlalom: CustomSlalom,
    # CustomAlignSlalomPathmarker.NoMeasurements: Surface,
    # CustomAlignSlalomPathmarker.TimedOut: CustomSlalom,

    # CustomSlalom.TimedOut: Surface,
    # CustomSlalom.SeenBinsPathmarker: CenterBinsPathmarker,

    # CenterBinsPathmarker.Centered: AlignBinsPathmarker,
    # CenterBinsPathmarker.TimedOut: AlignBinsPathmarker,

    # AlignBinsPathmarker.AlignedToBins: GoToOctagon,
    # AlignBinsPathmarker.NoMeasurements: Surface,
    # AlignBinsPathmarker.TimedOut: Surface,

    # GoToOctagon.Finished: Surface,
}

test_spin: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=0.5),

    Submerge.TimedOut: Spin.with_params(target_heave=0.5, speed=-0.5),
    Submerge.Submerged: Spin.with_params(target_heave=0.5, speed=-0.5),

    Spin.TimedOut: Surface
}

class SpeedForward(TimedState):
    class Finished(Outcome):
        pass

    timeout: float = 10.0
    speed: float = 1.0

    def handle_if_not_timedout(self) -> None:
        PIO.set_target_twist_surge(self.speed)
        return None
    
    def handle_once_timedout(self) -> Outcome:
        return self.Finished()

test_speed: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=0.5),

    Submerge.TimedOut: SpeedForward,
    Submerge.Submerged: SpeedForward,

    SpeedForward.Finished: Surface,
}
