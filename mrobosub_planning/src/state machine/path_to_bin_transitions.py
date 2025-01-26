from buoy_states import AlignBinsPathmarker
from bin_states import (
    ApproachBinOpen,
    ApproachBinClosed,
    CenterCameraToBin,
    Descend,
    CenterLeftDropper,
    DropMarker,
    Spin180,
)
from common_states import Start, Submerge, Surface, Stop
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: AlignBinsPathmarker,
    Submerge.TimedOut: AlignBinsPathmarker,

    AlignBinsPathmarker.AlignedToBins: ApproachBinOpen,
    AlignBinsPathmarker.NoMeasurements: ApproachBinOpen,
    AlignBinsPathmarker.TimedOut: Surface,

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
    Surface.Surfaced: Stop,
}
