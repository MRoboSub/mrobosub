#!/usr/bin/env python
from common_states import Start, Submerge, Surface, Stop
from gate_states import AlignGate, AlignSlalom, ApproachGate2, CenterSlalomPathmarker1, CenterSlalomPathmarker2, Spin, SpinFinish
from buoy_states import Slalom, CenterBinsPathmarker
from umrsm import TransitionMap

slalom_heave = 1.0
AlignSlalom = AlignSlalom.with_params(sway_speed=1.0, timeout=15.0, target_heave=slalom_heave)
Slalom = Slalom.with_params(target_heave=slalom_heave, surge_speed=1.0)

transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=1.),

    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.ReachedAngle: ApproachGate2,
    AlignGate.TimedOut: ApproachGate2,

    ApproachGate2.SeenPathmarker: CenterSlalomPathmarker1,
    ApproachGate2.TimedOut: Surface,

    CenterSlalomPathmarker1.Centered: Spin,
    CenterSlalomPathmarker1.TimedOut: Spin,

    Spin.TimedOut: SpinFinish,

    SpinFinish.Reached: CenterSlalomPathmarker2,
    SpinFinish.TimedOut: CenterSlalomPathmarker2,

    CenterSlalomPathmarker2.Centered: AlignSlalom,
    CenterSlalomPathmarker2.TimedOut: AlignSlalom,

    AlignSlalom.Finished: Slalom,

    Slalom.TimedOut: Surface,
    Slalom.SeenBinsPathmarker: CenterBinsPathmarker,

    CenterBinsPathmarker.Centered: Surface,
    CenterBinsPathmarker.TimedOut: Surface,
}
