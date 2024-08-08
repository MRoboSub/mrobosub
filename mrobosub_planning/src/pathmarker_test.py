from gate_states import AlignPathmarker, ApproachGate
from buoy_states import ApproachBuoyOpen
from common_states import Start, Submerge, Surface, Stop
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: AlignPathmarker,
    Submerge.TimedOut: AlignPathmarker,

    AlignPathmarker.AlignedToBuoy: Surface,
    AlignPathmarker.AlignedToBin: Surface,
    AlignPathmarker.TimedOut: Surface,

    Surface.Surfaced: Stop
}
