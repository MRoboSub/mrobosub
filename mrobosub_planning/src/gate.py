#!/usr/bin/env python
from common import Start, Submerge, Surface, Stop
from gate_task import AlignGate, ApproachGate, ApproachGateImage
from buoy_task import ApproachBuoyOpen, OldApproachBuoyClosed, FindGlyph, FallBack, PassBuoy, Pause, ContingencyApproach, ContingencySubmerge, Ascend


transitions = {
    Start.Complete: Submerge,

    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.SeenGateImage: ApproachGateImage,
    ApproachGate.TimedOut: Surface,

    # FoundBuoyPathMarker: AlignPathMarker,

    # ApproachGateImage.GoneThroughGate: FallBackTurn,
    ApproachGateImage.TimedOut: ApproachBuoyOpen,

    # AlignPathMarker.Unaligned: AlignPathMarker,
    # AlignPathMarker.Aligned: ApproachBuoyOpen,
    # AlignPathMarker.TimedOut: ApproachBuoyOpen,

    # FallBackTurn.Unaligned: FallBackTurn,
    # FallBackTurn.Aligned: ApproachBuoyOpen,

    # SeenGlyph: OldApproachBuoyClosed,
    # HitBuoyFirst: FindGlyph,
    # HitBuoySecond: FallBack,

    ApproachBuoyOpen.SeenGlyph: OldApproachBuoyClosed,
    ApproachBuoyOpen.TimedOut: Surface,

    OldApproachBuoyClosed.HitBuoyFirst: FindGlyph,
    OldApproachBuoyClosed.HitBuoySecond: FallBack,
    OldApproachBuoyClosed.TimedOut: Surface,

    FindGlyph.SeenGlyph: OldApproachBuoyClosed,
    FindGlyph.TimedOut: Pause,

    Pause.SeenGlyph: OldApproachBuoyClosed,
    Pause.TimedOut: ContingencySubmerge,

    ContingencySubmerge.SeenGlyph: OldApproachBuoyClosed,
    ContingencySubmerge.Submerged: ContingencyApproach,

    ContingencyApproach.HitBuoySecond: FallBack,
    ContingencyApproach.TimedOut: Surface,
    
    FallBack.TimedOut: Ascend,

    Ascend.Reached: PassBuoy,
    Ascend.TimedOut: PassBuoy,

    PassBuoy.TimedOut: Surface,

    Surface.Surfaced: Stop     
}
