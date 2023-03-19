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
    ApproachGate.FoundPathMarker: AlignPathMarker,
    ApproachGate.TimedOut: FallBackTurn,

    #Scan.NotFound: Scan,
    #Scan.FoundPathMarker: AlignPathMarker,
    #Scan.TimedOut: FallBackTurn,

    FallBackTurn.Unaligned: FallbackTurn,
    FallBackTurn.Aligned: ApproachBuoy,

    AlignPathMarker.Unaligned: AlignPathMarker,
    AlignPathMarker.Aligned: ApproachBuoy,

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
