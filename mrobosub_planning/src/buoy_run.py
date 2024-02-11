#!/usr/bin/env python
from common import Start, Submerge, Surface, Stop
from buoy_task import ApproachBuoyOpen, CenterHeaveGlyph, CenterYawGlyph, FindGlyph, FallBack, ContingencySubmerge, ContingencyApproach, Pause
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: ApproachBuoyOpen,
    Submerge.TimedOut: ApproachBuoyOpen,

    ApproachBuoyOpen.SeenGlyph: CenterHeaveGlyph,
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveGlyph.Centered: CenterYawGlyph,
    CenterHeaveGlyph.TimedOut: CenterYawGlyph,

    CenterYawGlyph.HitBuoyFirst: FindGlyph,
    CenterYawGlyph.HitBuoySecond: FallBack,
    CenterYawGlyph.TimedOut: Surface,

    FindGlyph.SeenGlyph: CenterHeaveGlyph,
    FindGlyph.TimedOut: Pause,

    Pause.SeenGlyph: CenterHeaveGlyph,
    Pause.TimedOut: ContingencySubmerge,
    Pause.Pausing: Pause,

    ContingencySubmerge.SeenGlyph: CenterHeaveGlyph,
    ContingencySubmerge.Submerged: ContingencyApproach,

    ContingencyApproach.HitBuoySecond: FallBack,
    ContingencyApproach.TimedOut: Surface,

    FallBack.TimedOut: Surface,

    Surface.Surfaced: Stop
}
