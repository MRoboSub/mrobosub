from gate_states import AlignPathmarker, ApproachGate
from buoy_states import ApproachBuoyOpen
from common_states import Start, Submerge, Surface, Stop
from gate_states import AlignPathmarker
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: AlignPathmarker,
    Submerge.TimedOut: AlignPathmarker,

    AlignPathmarker.Aligned: ApproachBin,
    AlignPathmarker.TimedOut: Surface,

    ApproachBin.Reached: CenterCameraToBin,
    ApproachBin.TimedOut: Surface,

    CenterCameraToBin.Reached: Descend,
    CenterCameraToBin.TimedOut: Surface,

    Descend.Reached: Surface,
    Descend.TimedOut: Surface,

    Surface.Surfaced: Stop
}
