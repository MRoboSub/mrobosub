#!/usr/bin/env python
from common import *
from gate_task import *
from buoy_task import *
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.Unaligned: AlignGate,
    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.SeenGateImage: ApproachGateImage,
    ApproachGate.Unreached: ApproachGate,
    ApproachGate.TimedOut: Spin,

    # SeenGateImage: ApproachGateImage,

    ApproachGateImage.SeenGateImage: ApproachGateImage,
    ApproachGateImage.GoneThroughGate: Spin,
    ApproachGateImage.TimedOut: Surface,

    # # should be unused
    # AlignPathMarker.Unaligned: AlignPathMarker,
    # AlignPathMarker.Aligned: ApproachBuoyOpen,
    # AlignPathMarker.TimedOut: ApproachBuoyOpen,

    Spin.Unreached: Spin,
    Spin.TimedOut: SpinFinish,

    SpinFinish.Unreached: SpinFinish,
    SpinFinish.Reached: ApproachBuoyOpen,
    SpinFinish.TimedOut: ApproachBuoyOpen,

    # # should be unused
    # FallBackTurn.Unaligned: FallBackTurn,
    # FallBackTurn.Aligned: ApproachBuoyOpen,

    ApproachBuoyOpen.SeenGlyph: CenterHeaveGlyph,
    ApproachBuoyOpen.GlyphNotSeen: ApproachBuoyOpen, 
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveGlyph.NotCentered: CenterHeaveGlyph,
    CenterHeaveGlyph.Centered: CenterYawGlyph,
    CenterHeaveGlyph.TimedOut: CenterYawGlyph,

    CenterYawGlyph.HitBuoyFirst: FindGlyph,
    CenterYawGlyph.HitBuoySecond: FallBack,
    CenterYawGlyph.NotReached: CenterYawGlyph,
    CenterYawGlyph.TimedOut: Surface,

    FindGlyph.SeenGlyph: CenterHeaveGlyph,
    FindGlyph.GlyphNotSeen: FindGlyph,
    FindGlyph.TimedOut: Pause,

    Pause.SeenGlyph: CenterHeaveGlyph,
    Pause.TimedOut: ContingencySubmerge,

    ContingencySubmerge.SeenGlyph: CenterHeaveGlyph,
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
