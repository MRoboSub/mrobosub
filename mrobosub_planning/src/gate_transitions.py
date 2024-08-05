#!/usr/bin/env python
from common_states import Start, Submerge, Surface, Stop
from gate_states import AlignGate, AlignPathmarker, ApproachGate, ApproachGateImage
# from buoy_states import ApproachBuoyOpen, OldApproachBuoyClosed, FindGlyph, FallBack, PassBuoy, Pause, ContingencyApproach, ContingencySubmerge, Ascend


transitions = {
    Start.Complete: Submerge,

    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.SeenGateImage: ApproachGateImage,
    ApproachGate.TimedOut: Surface,

    # AlignPathMarker.Aligned: ApproachBuoyOpen,
    # AlignPathMarker.SeenGlyph: OldApproachBuoyClosed,
    # AlignPathMarker.TimedOut: ApproachBuoyOpen,

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

    Surface.Surfaced: Stop     
}
