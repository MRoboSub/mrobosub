from gate_task import ApproachGate
from buoy_task import ApproachBuoyOpen
from common import Start, Submerge, Surface, Stop
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: ApproachGate,
    Submerge.TimedOut: ApproachGate,

    # ApproachGate.Unreached: AlignPathMarker,
    ApproachGate.TimedOut: Surface,
    # FoundBuoyPathMarker: AlignPathMarker,

    # AlignPathMarker.Unaligned: AlignPathMarker,
    # AlignPathMarker.Aligned: ApproachBuoyOpen,
    # AlignPathMarker.TimedOut: Surface,

    ApproachBuoyOpen.SeenGlyph: Surface,
    ApproachBuoyOpen.TimedOut: Surface,

    # SeenGlyph: Surface,
    # HitBuoyFirst: Surface,

    Surface.Surfaced: Stop
}
