#!/usr/bin/env python
from common_states import Start, Submerge, Surface, Stop
from gate_states import AlignGate, AlignBuoyPathmarker, ApproachGate, Spin, ApproachGateImage, SpinFinish, GuessBuoyAngle
from buoy_states import ApproachBuoyOpen, AlignBinsPathmarker, BuoyPause, CenterHeaveBuoy, CenterYawBuoy, CenterYawBuoyDiscrete, ZedPause
from circumnavigate_states import CircumnavigateOpenDiscreteDiamondTurns, CircumnavigateOpenDiscreteMove
from bin_states import ApproachBinOpen, ApproachBinClosed, CenterCameraToBin, Descend, CenterLeftDropper, DropMarker, Spin180
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    # Submerge.Submerged: AlignGate,
    # Submerge.TimedOut: AlignGate,

    Submerge.Submerged: ZedPause,
    Submerge.TimedOut: ZedPause,
    

    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.SeenGateImage: ApproachGateImage,
    ApproachGate.TimedOut: Spin,

    ApproachGateImage.GoneThroughGate: Spin,
    ApproachGateImage.TimedOut: Surface,

    Spin.TimedOut: SpinFinish,

    SpinFinish.Reached: AlignBuoyPathmarker,
    SpinFinish.TimedOut: AlignBuoyPathmarker,
    
    AlignBuoyPathmarker.AlignedToBuoy: ZedPause, #ApproachBuoyOpen,
    AlignBuoyPathmarker.NoMeasurements: GuessBuoyAngle,
    AlignBuoyPathmarker.TimedOut: GuessBuoyAngle,

    GuessBuoyAngle.Reached: ZedPause,
    GuessBuoyAngle.TimedOut: ZedPause,

    ZedPause.TimedOut: ApproachBuoyOpen,

    ApproachBuoyOpen.SeenBuoy: CenterHeaveBuoy,
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveBuoy.Centered: CenterYawBuoy,
    CenterHeaveBuoy.TimedOut: CenterYawBuoy,

    CenterYawBuoyDiscrete.CloseToBuoy: BuoyPause, # this is where movement of around buoy will connect
    CenterYawBuoyDiscrete.TimedOut: Surface, #could do passBuoy instead of surface as well

    BuoyPause.TimedOut: CircumnavigateOpenDiscreteDiamondTurns,

    CircumnavigateOpenDiscreteDiamondTurns.FinishedStep: CircumnavigateOpenDiscreteMove,
    CircumnavigateOpenDiscreteDiamondTurns.Complete: Surface, #use pathmarker to align to bin
    CircumnavigateOpenDiscreteDiamondTurns.TimedOut: Surface,

    CircumnavigateOpenDiscreteMove.FinishedStep: CircumnavigateOpenDiscreteDiamondTurns,

    AlignBinsPathmarker.AlignedToBins: ApproachBinOpen, #this is where we will go to bin
    AlignBinsPathmarker.NoMeasurements: Surface,
    AlignBinsPathmarker.TimedOut: ApproachBinOpen,

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
