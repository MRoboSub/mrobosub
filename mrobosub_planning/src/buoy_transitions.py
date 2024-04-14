#!/usr/bin/env python
from common_states import Start, Submerge, Surface, Stop
from buoy_states import ApproachBuoyOpen, CenterHeaveBuoy, CenterYawBuoy
from circumnavigate_states import CircumnavigateOpenDiscreteDiamondTurns, CircumnavigateOpenDiscreteMove
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: ApproachBuoyOpen,
    Submerge.TimedOut: ApproachBuoyOpen,

    ApproachBuoyOpen.SeenBuoy: CenterHeaveBuoy,
    ApproachBuoyOpen.TimedOut: Surface,

    CenterHeaveBuoy.Centered: CenterYawBuoy,
    CenterHeaveBuoy.TimedOut: Surface,

    CenterYawBuoy.CloseToBuoy: CircumnavigateOpenDiscreteDiamondTurns,
    CenterYawBuoy.TimedOut: Surface,

    CircumnavigateOpenDiscreteDiamondTurns.FinishedStep: CircumnavigateOpenDiscreteMove,
    CircumnavigateOpenDiscreteDiamondTurns.Complete: Surface, #this means completed successfully
    CircumnavigateOpenDiscreteDiamondTurns.TimedOut: Surface,

    CircumnavigateOpenDiscreteMove.FinishedStep: CircumnavigateOpenDiscreteDiamondTurns,

    Surface.Surfaced: Stop
}
