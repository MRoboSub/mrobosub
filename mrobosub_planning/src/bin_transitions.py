from common_states import *
from bin_states import *
from umrsm import TransitionMap


transitions:TransitionMap = {
    Start.Complete: ApproachBinOpen,#Submerge,

    Submerge.Submerged: ApproachBinOpen,
    Submerge.TimedOut: ApproachBinClosed,

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

    Spin180.Reached: DropMarker, #Surface,#
    Spin180.TimedOut: Surface,

    Surface.Surfaced: Stop
}
