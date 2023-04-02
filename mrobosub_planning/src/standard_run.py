#!/usr/bin/env python
from gate_task import *
from buoy_task import *


transitions = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.Unaligned: AlignGate,
    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.Unreached: ApproachGate,
    ApproachGate.TimedOut: FallBackTurn,

    SeenGateImage: ApproachGateImage,
    FoundBuoyPathMarker: AlignPathMarker,

    ApproachGateImage.GoneThroughGate: FallBackTurn,

    AlignPathMarker.Unaligned: AlignPathMarker,
    AlignPathMarker.Aligned: ApproachBuoyOpen,
    AlignPathMarker.TimedOut: FallBackTurn,

    FallBackTurn.Unaligned: FallbackTurn,
    FallBackTurn.Aligned: ApproachBuoyOpen,

    ApproachBuoyOpen.BuoyNotSeen: ApproachBuoyOpen,
    ApproachBuoyOpen.SeenBuoy: ApproachBuoyClosed,
    ApproachBuoyOpen.TimedOut: FallBack,

    ApproachBuoyClosed.NotReached: ApproachBuoyClosed,
    ApproachBuoyClosed.HitBuoy: FallBack,
    ApproachBuoyClosed.TimedOut: FallBack,

    FallBack.NotReached: FallBack,
    FallBack.TimedOut: Ascend,

    Ascend.NotReached: Ascend,
    Ascend.Reached: PassBuoy,
    Ascend.TimedOut: PassBuoy,

    PassBuoy.NotReached: PassBuoy,
    PassBuoy.TimedOut: Surface,

    Surface.Submerged: Surface,
    Surface.Surfaced: Stop     
}
