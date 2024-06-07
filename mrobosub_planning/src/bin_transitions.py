from common_states import *
from bin_states import *
from umrsm import TransitionMap


transitions:TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: ApproachBin,
    Submerge.TimedOut: ApproachBin,

    ApproachBin.Reached: CenterCameraToBin,
    ApproachBin.TimedOut:  CenterCameraToBin, #Surface,

    CenterCameraToBin.Reached: Descend,
    CenterCameraToBin.TimedOut:  Descend,#Surface,
    
    Descend.Reached: CenterLeftDropper,
    Descend.TimedOut: CenterLeftDropper,#Surface, #

    CenterLeftDropper.Reached: DropMarker,
    CenterLeftDropper.TimedOut: DropMarker,#Surface, #

    DropMarker.DroppedLeft: Spin180,
    DropMarker.TimedOut: Spin180,#Surface, #
    DropMarker.DroppedRight: Surface,

    Spin180.Reached: DropMarker, #Surface,#
    Spin180.TimedOut: Surface,

    Surface.Surfaced: Stop
}
