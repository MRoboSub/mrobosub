#!/usr/bin/env python
from common_states import Start, Submerge, Surface, Stop
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=1.),

    Submerge.Submerged: Surface,
    Submerge.TimedOut: Surface,

    Surface.Surfaced: Stop
}
