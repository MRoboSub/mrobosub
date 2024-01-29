#!/usr/bin/env python
from common import *
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
    ApproachGate.TimedOut: Surface,

    SeenGateImage: ApproachGateImage,
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
    ApproachBuoyOpen.GlyphNotSeen: ApproachBuoyOpen, 
    ApproachBuoyOpen.TimedOut: Surface,

    OldApproachBuoyClosed.HitBuoyFirst: FindGlyph,
    OldApproachBuoyClosed.HitBuoySecond: FallBack,
    OldApproachBuoyClosed.NotReached: OldApproachBuoyClosed,
    OldApproachBuoyClosed.TimedOut: Surface,

    FindGlyph.SeenGlyph: OldApproachBuoyClosed,
    FindGlyph.GlyphNotSeen: FindGlyph,
    FindGlyph.TimedOut: Pause,

    Pause.SeenGlyph: OldApproachBuoyClosed,
    Pause.TimedOut: ContingencySubmerge,

    ContingencySubmerge.SeenGlyph: OldApproachBuoyClosed,
    ContingencySubmerge.Submerging: ContingencySubmerge,
    ContingencySubmerge.Submerged: ContingencyApproach,

    ContingencyApproach.HitBuoySecond: FallBack,
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
