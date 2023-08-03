
from gate_task import *
from buoy_task import *
from common import *


transitions = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: AlignPathMarker,
    Submerge.TimedOut: AlignPathMarker,

    AlignPathMarker.Unaligned: AlignPathMarker,
    AlignPathMarker.Aligned: ApproachBuoyOpen,
    AlignPathMarker.TimedOut: Surface,

    ApproachBuoyOpen.GlyphNotSeen: ApproachBuoyOpen, 
    ApproachBuoyOpen.Timeout: Surface,

    SeenGlyph: Surface
    HitBuoyFirst: Surface,

    Surface.Submerged: Surface,
    Surface.Surfaced: Stop
}
