from gate_states import AlignPathmarker
from bin_states import ApproachBinOpen, ApproachBinClosed, CenterCameraToBin, Descend, CenterLeftDropper, DropMarker, Spin180
from common_states import Start, Submerge, Surface, Stop
from gate_states import AlignPathmarker
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: AlignPathmarker,
    Submerge.TimedOut: AlignPathmarker,

    AlignPathmarker.AlignedToBuoy: ApproachBinOpen,
    AlignPathmarker.AlignedToBin: ApproachBinOpen,
    AlignPathmarker.TimedOut: Surface,

    ApproachBinOpen.SeenBin: ApproachBinClosed,
    ApproachBinOpen.TimedOut: Surface,

    ApproachBinClosed.Reached: CenterCameraToBin,
    ApproachBinClosed.TimedOut: Surface,

    CenterCameraToBin.Reached: CenterLeftDropper, # maybe this after descend?
    CenterCameraToBin.TimedOut: Surface,

    CenterLeftDropper.Reached: Descend,
    CenterLeftDropper.TimedOut: Surface,

    Descend.Reached: DropMarker,
    Descend.TimedOut: Surface,

    DropMarker.DroppedLeft: Spin180,
    DropMarker.TimedOut: Surface,

    Spin180.Reached: DropMarker,
    Spin180.TimedOut: Surface,

    DropMarker.DroppedRight: Surface,

    Surface.Surfaced: Stop
}
