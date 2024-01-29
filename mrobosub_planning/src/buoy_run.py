#!/usr/bin/env python
from common import *
from gate_task import *
from buoy_task import *
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: ApproachBuoyOpen,
    Submerge.TimedOut: ApproachBuoyOpen,

    # SeenGlyph: CenterHeaveGlyph,
    # HitBuoyFirst: FindGlyph,
    # HitBuoySecond: FallBack,

    ApproachBuoyOpen.SeenGlyph: CenterHeaveGlyph,
    ApproachBuoyOpen.GlyphNotSeen: ApproachBuoyOpen, 
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveGlyph.NotCentered: CenterHeaveGlyph,
    CenterHeaveGlyph.Centered: CenterYawGlyph,
    CenterHeaveGlyph.TimedOut: CenterYawGlyph,

    CenterYawGlyph.HitBuoyFirst: FindGlyph,
    CenterYawGlyph.HitBuoySecond: FallBack,
    CenterYawGlyph.NotReached: CenterYawGlyph,
    CenterYawGlyph.TimedOut: Surface,

    FindGlyph.SeenGlyph: CenterHeaveGlyph,
    FindGlyph.GlyphNotSeen: FindGlyph,
    FindGlyph.TimedOut: Pause,

    Pause.SeenGlyph: CenterHeaveGlyph,
    Pause.TimedOut: ContingencySubmerge,

    ContingencySubmerge.SeenGlyph: CenterHeaveGlyph,
    ContingencySubmerge.Submerging: ContingencySubmerge,
    ContingencySubmerge.Submerged: ContingencyApproach,

    ContingencyApproach.HitBuoySecond: FallBack,
    ContingencyApproach.Approaching: ContingencyApproach,
    ContingencyApproach.TimedOut: Surface,

    FallBack.NotReached: FallBack,
    FallBack.TimedOut: Surface,

    Surface.Submerged: Surface,
    Surface.Surfaced: Stop
}
