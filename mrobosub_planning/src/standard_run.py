#!/usr/bin/env python
from common import Start, Submerge, Surface, Stop
from gate_task import AlignGate, AlignPathMarker, ApproachGate, Spin, ApproachGateImage, SpinFinish
from buoy_task import ApproachBuoyOpen, CenterHeaveGlyph, CenterYawGlyph, FallBack, FindGlyph, PassBuoy, Pause, ContingencyApproach, ContingencySubmerge, Ascend
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.SeenGateImage: ApproachGateImage,
    ApproachGate.TimedOut: Spin,

    ApproachGateImage.GoneThroughGate: Spin,
    ApproachGateImage.TimedOut: Surface,

    # AlignPathMarker.SeenGlyph: CenterHeaveGlyph,
    # AlignPathMarker.Aligned: ApproachBuoyOpen,
    # AlignPathMarker.TimedOut: ApproachBuoyOpen,

    Spin.TimedOut: SpinFinish,

    SpinFinish.Reached: ApproachBuoyOpen,
    SpinFinish.TimedOut: ApproachBuoyOpen,

    ApproachBuoyOpen.SeenGlyph: CenterHeaveGlyph,
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveGlyph.Centered: CenterYawGlyph,
    CenterHeaveGlyph.TimedOut: CenterYawGlyph,

    CenterYawGlyph.HitBuoyFirst: FindGlyph,
    CenterYawGlyph.HitBuoySecond: FallBack,
    CenterYawGlyph.TimedOut: Surface,

    FindGlyph.SeenGlyph: CenterHeaveGlyph,
    FindGlyph.TimedOut: Pause,

    Pause.SeenGlyph: CenterHeaveGlyph,
    Pause.TimedOut: ContingencySubmerge,

    ContingencySubmerge.SeenGlyph: CenterHeaveGlyph,
    ContingencySubmerge.Submerged: ContingencyApproach,

    ContingencyApproach.HitBuoySecond: FallBack,
    ContingencyApproach.TimedOut: Surface,

    FallBack.TimedOut: Ascend,

    Ascend.Reached: PassBuoy,
    Ascend.TimedOut: PassBuoy,

    PassBuoy.TimedOut: Surface,

    Surface.Surfaced: Stop     
}
