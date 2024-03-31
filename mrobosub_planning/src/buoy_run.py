#!/usr/bin/env python
from common import Start, Submerge, Surface, Stop
from buoy_task import ApproachBuoyOpen, CenterHeaveBuoy, CenterYawBuoy
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: ApproachBuoyOpen,
    Submerge.TimedOut: ApproachBuoyOpen,

    ApproachBuoyOpen.SeenBuoy: CenterHeaveBuoy,
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveBuoy.Centered: CenterYawBuoy,
    CenterHeaveBuoy.TimedOut: CenterYawBuoy,

    CenterYawBuoy.CloseToBuoy: Surface,
    CenterYawBuoy.TimedOut: Surface,

    Surface.Surfaced: Stop
}
