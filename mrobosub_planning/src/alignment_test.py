
from gate_task import *
from buoy_task import *
from common import *
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: ApproachGate,
    Submerge.TimedOut: ApproachGate,

    # ApproachGate.Unreached: AlignPathMarker,
    ApproachGate.TimedOut: Surface,
    # FoundBuoyPathMarker: AlignPathMarker,

    # AlignPathMarker.Unaligned: AlignPathMarker,
    # AlignPathMarker.Aligned: ApproachBuoyOpen,
    # AlignPathMarker.TimedOut: Surface,

    ApproachBuoyOpen.GlyphNotSeen: ApproachBuoyOpen, 
    ApproachBuoyOpen.TimedOut: Surface,

    SeenGlyph: Surface,
    HitBuoyFirst: Surface,

    Surface.Submerged: Surface,
    Surface.Surfaced: Stop
}
