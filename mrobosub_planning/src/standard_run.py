#!/usr/bin/env python
from common_states import Start, Submerge, Surface, Stop
from gate_states import AlignGate, AlignPathmarker, ApproachGate, Spin, ApproachGateImage, SpinFinish
from buoy_states import ApproachBuoyOpen, CenterHeaveBuoy, CenterYawBuoy
from circumnavigate_states import CircumnavigateOpenDiscreteDiamondTurns, CircumnavigateOpenDiscreteMove
from bin_states import ApproachBinOpen, ApproachBinClosed, CenterCameraToBin, Descend, CenterLeftDropper, DropMarker, Spin180
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

    Spin.TimedOut: SpinFinish,

    SpinFinish.Reached: AlignPathmarker,
    SpinFinish.TimedOut: AlignPathmarker,
    
    AlignPathmarker.AlignedToBuoy: ApproachBuoyOpen,
    AlignPathmarker.TimedOutBuoy: ApproachBuoyOpen,

    ApproachBuoyOpen.SeenBuoy: CenterHeaveBuoy,
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveBuoy.Centered: CenterYawBuoy,
    CenterHeaveBuoy.TimedOut: CenterYawBuoy,

    CenterYawBuoy.CloseToBuoy: CircumnavigateOpenDiscreteDiamondTurns, # this is where movement of around buoy will connect
    CenterYawBuoy.TimedOut: Surface, #could do passBuoy instead of surface as well

    CircumnavigateOpenDiscreteDiamondTurns.FinishedStep: CircumnavigateOpenDiscreteMove,
    CircumnavigateOpenDiscreteDiamondTurns.Complete: AlignPathmarker, #use pathmarker to align to bin
    CircumnavigateOpenDiscreteDiamondTurns.TimedOut: Surface,

    CircumnavigateOpenDiscreteMove.FinishedStep: CircumnavigateOpenDiscreteDiamondTurns,

    AlignPathmarker.AlignedToBin: ApproachBinOpen, #this is where we will go to bin
    AlignPathmarker.TimedOutBin: ApproachBinOpen,

    ApproachBinOpen.SeenBin: ApproachBinClosed,
    ApproachBinOpen.TimedOut: Surface,

    ApproachBinClosed.Reached: CenterCameraToBin,
    ApproachBinClosed.TimedOut: Surface,

    CenterCameraToBin.Reached: Descend,
    CenterCameraToBin.TimedOut: Surface,
    
    Descend.Reached: CenterLeftDropper,
    Descend.TimedOut: Surface,

    CenterLeftDropper.Reached: DropMarker,
    CenterLeftDropper.TimedOut: Surface,

    DropMarker.DroppedLeft: Spin180,
    DropMarker.TimedOut: Surface,
    DropMarker.DroppedRight: Surface,

    Spin180.Reached: DropMarker,
    Spin180.TimedOut: Surface,

    Surface.Surfaced: Stop     
}
