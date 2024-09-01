from common_states import *
from bin_states import *
from umrsm import TransitionMap


transitions:TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: ApproachBinOpen,
    Submerge.TimedOut: ApproachBinOpen,

    ApproachBinOpen.SeenBin: CenterCameraToBin,
    ApproachBinOpen.TimedOut: Surface,

    ApproachBinClosed.Reached: CenterCameraToBin,
    ApproachBinClosed.TimedOut: Surface,

    CenterCameraToBin.Reached: CenterLeftDropper, # Descend,
    CenterCameraToBin.TimedOut: CenterLeftDropper, # Surface

    Descend.Reached: CenterLeftDropper,
    Descend.TimedOut: Surface,

    CenterLeftDropper.Reached: DropMarker,
    CenterLeftDropper.TimedOut: Surface,

    DropMarker.DroppedLeft: Spin180,
    DropMarker.TimedOut: Surface,
    DropMarker.DroppedRight: Surface,

    Spin180.Reached: DropMarker,
    Spin180.TimedOut: Surface,

    Surface.Surfaced: Stop,
}
