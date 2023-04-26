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

    AlignPathMarker.Unaligned: ApproachBuoyOpen,
    AlignPathMarker.Aligned: AlignPathMarker,
    AlignPathMarker.TimedOut: ApproachBuoyOpen,

    FallBackTurn.Unaligned: FallbackTurn,
    FallBackTurn.Aligned: ApproachBuoyOpen,

    SeenGlyph:ApproachBuoyClosed,
    HitBuoySecond: FallBack,
    HitBuoyFirst: FindGlyph,

    ApproachBuoyOpen.GlyphNotSeen: ApproachBuoyOpen, 
    ApproachBuoyOpen.HitBuoy: FindGlyph,
    ApproachBuoyOpen.Timeout: Surface,

    ApproachBuoyClosed.NotReached: ApproachBuoyClosed,
    ApproachBuoyClosed.Timeout: Surface,

    FindGlyph.GlyphNotSeen: FindGlyph,
    FindGlyph.TimedOut: Pause,
    
    Pause.FoundGlyph: ApproachBuoyClosed,
    Pause.TimedOut: ContingencySubmerge,
    
    ContingencySubmerge.Submerging: ContingencySubmerge,
    ContingencySubmerge.Submerged: ContingencyApproach,

    ContingencyApproach.Approaching: ContingencyApproach,
    ContingencyApproach.TimedOut: Surface,
    
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
