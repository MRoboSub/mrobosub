#!/usr/bin/env python
from common_states import Start, Stop, Submerge, Surface
from gate_states import AlignBuoyPathmarker, AlignGate, ApproachGate, ApproachGateImage
from umrsm import TransitionMap

# from buoy_states import ApproachBuoyOpen, OldApproachBuoyClosed, FindGlyph, FallBack, PassBuoy, Pause, ContingencyApproach, ContingencySubmerge, Ascend


transitions: TransitionMap = {
    Start.Complete: Submerge,
    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,
    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,
    ApproachGate.SeenGateImage: ApproachGateImage,
    ApproachGate.TimedOut: Surface,
    # AlignBuoyPathmarker.Aligned: ApproachBuoyOpen,
    # AlignBuoyPathmarker.SeenGlyph: OldApproachBuoyClosed,
    # AlignBuoyPathmarker.TimedOut: ApproachBuoyOpen,
    # ApproachGateImage.TimedOut: ApproachBuoyOpen,
    # ApproachBuoyOpen.SeenGlyph: OldApproachBuoyClosed,
    # ApproachBuoyOpen.TimedOut: Surface,
    # OldApproachBuoyClosed.HitBuoyFirst: FindGlyph,
    # OldApproachBuoyClosed.HitBuoySecond: FallBack,
    # OldApproachBuoyClosed.TimedOut: Surface,
    # FindGlyph.SeenGlyph: OldApproachBuoyClosed,
    # FindGlyph.TimedOut: Pause,
    # Pause.SeenGlyph: OldApproachBuoyClosed,
    # Pause.TimedOut: ContingencySubmerge,
    # ContingencySubmerge.SeenGlyph: OldApproachBuoyClosed,
    # ContingencySubmerge.Submerged: ContingencyApproach,
    # ContingencyApproach.HitBuoySecond: FallBack,
    # ContingencyApproach.TimedOut: Surface,
    # FallBack.TimedOut: Ascend,
    # Ascend.Reached: PassBuoy,
    # Ascend.TimedOut: PassBuoy,
    # PassBuoy.TimedOut: Surface,
    Surface.Surfaced: Stop,
}
