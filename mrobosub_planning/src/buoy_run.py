#!/usr/bin/env python
from common import *
from gate_task import *
from buoy_task import *


transitions = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: ApproachBuoyOpen,
    Submerge.TimedOut: ApproachBuoyOpen,

    SeenGlyph: CenterHeaveGlyph,
    HitBuoyFirst: FindGlyph,
    HitBuoySecond: FallBack,

    ApproachBuoyOpen.GlyphNotSeen: ApproachBuoyOpen, 
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveGlyph.NotCentered: CenterHeaveGlyph,
    CenterHeaveGlyph.Centered: CenterYawGlyph,
    CenterHeaveGlyph.TimedOut: CenterYawGlyph,

    CenterYawGlyph.NotReached: CenterYawGlyph,
    CenterYawGlyph.TimedOut: Surface,

    FindGlyph.GlyphNotSeen: FindGlyph,
    FindGlyph.TimedOut: Pause,

    Pause.TimedOut: ContingencySubmerge,

    ContingencySubmerge.Submerging: ContingencySubmerge,
    ContingencySubmerge.Submerged: ContingencyApproach,

    ContingencyApproach.Approaching: ContingencyApproach,
    ContingencyApproach.TimedOut: Surface,

    FallBack.NotReached: FallBack,
    FallBack.TimedOut: Surface,

    Surface.Submerged: Surface,
    Surface.Surfaced: Stop
}
