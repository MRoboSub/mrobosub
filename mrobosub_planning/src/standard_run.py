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
    ApproachGate.TimedOut: Spin,

    SeenGateImage: ApproachGateImage,

    ApproachGateImage.GoneThroughGate: Spin,
    ApproachGateImage.TimedOut: Surface,

    # should be unused
    AlignPathMarker.Unaligned: AlignPathMarker,
    AlignPathMarker.Aligned: ApproachBuoyOpen,
    AlignPathMarker.TimedOut: ApproachBuoyOpen,

    Spin.Unreached: Spin,
    Spin.TimedOut: SpinFinish,

    SpinFinish.Unreached: SpinFinish,
    SpinFinish.Reached: ApproachBuoyOpen,
    SpinFinish.TimedOut: ApproachBuoyOpen,

    # should be unused
    FallBackTurn.Unaligned: FallBackTurn,
    FallBackTurn.Aligned: ApproachBuoyOpen,

    SeenGlyph: CenterHeaveGlyph,
    HitBuoyFirst: Backup,
    HitBuoySecond: FallBack,

    ApproachBuoyOpen.GlyphNotSeen: ApproachBuoyOpen, 
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveGlyph.NotCentered: CenterHeaveGlyph,
    CenterHeaveGlyph.Centered: CenterYawGlyph,
    CenterHeaveGlyph.TimedOut: CenterYawGlyph,

    CenterYawGlyph.NotReached: CenterYawGlyph,
    CenterYawGlyph.TimedOut: Surface,

    Backup.GlyphNotSeen: Backup,
    Backup.TimedOut: Pause,
    
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
