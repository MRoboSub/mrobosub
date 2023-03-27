#!/usr/bin/env python
from common_states import Start, Submerge, Surface, Stop
from gate_states import AlignGate, AlignPathmarker, ApproachGate, Spin, ApproachGateImage, SpinFinish
from buoy_states import ApproachBuoyOpen, CenterHeaveBuoy, CenterYawBuoy
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

    ApproachBuoyOpen.SeenBuoy: CenterHeaveBuoy,
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveBuoy.Centered: CenterYawBuoy,
    CenterHeaveBuoy.TimedOut: CenterYawBuoy,

    CenterYawBuoy.CloseToBuoy: Surface, # this is where movement of around buoy will connect
    CenterYawBuoy.TimedOut: Surface, #could do passBuoy instead of surface as well

    Surface.Surfaced: Stop     
}
