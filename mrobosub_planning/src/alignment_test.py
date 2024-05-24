from gate_states import AlignPathmarker, ApproachGate
from buoy_states import ApproachBuoyOpen
from common_states import Start, Submerge, Surface, Stop
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: ApproachGate,
    Submerge.TimedOut: ApproachGate,

    ApproachGate.TimedOut: Surface,

    # AlignPathMarker.Aligned: ApproachBuoyOpen,
    # AlignPathMarker.SeenGlyph: Surface,
    # AlignPathMarker.TimedOut: Surface,

    ApproachBuoyOpen.SeenBuoy: Surface,
    ApproachBuoyOpen.TimedOut: Surface,

    Surface.Surfaced: Stop
}
