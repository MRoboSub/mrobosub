#!/usr/bin/env python
from mrobosub_planning.common_states import Outcome, Start, Submerge, Surface, Stop, Surge, AlignToYaw
from mrobosub_planning.umrsm import TransitionMap

def make_move_forward():
    class SurgeExtender(Surge):
        class Surged(Outcome):
            pass
        class TimedOut(Outcome):
            pass
    
    class AlignExtender(AlignToYaw):
        class Aligned(Outcome):
            pass
        class TimedOut(Outcome):
            pass
    
    return SurgeExtender, AlignExtender

Surge1, Align1 = make_move_forward()
Surge2, Align2 = make_move_forward()
Surge3, Align3 = make_move_forward()
Surge4, Align4 = make_move_forward()
Surge5, Align5 = make_move_forward()
Surge6, _ = make_move_forward()


transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=1.),

    Submerge.Submerged: Surge1.with_params(distance=20.),  # object distance
    Submerge.TimedOut: Surface,

    Surge1.Surged: Align1.with_params(target_yaw=45.),
    Align1.Aligned: Surge2.with_params(distance=10.),
    Align1.TimedOut: Surface,

    Surge2.Surged: Align2.with_params(target_yaw=315.),
    Align2.Aligned: Surge3.with_params(distance=10.),
    Align2.TimedOut: Surface,

    Surge3.Surged: Align3.with_params(target_yaw=225.),
    Align3.Aligned: Surge4.with_params(distance=10.),
    Align3.TimedOut: Surface,

    Surge4.Surged: Align4.with_params(target_yaw=135.),
    Align4.Aligned: Surge5.with_params(distance=10.),
    Align4.TimedOut: Surface,

    Surge5.Surged: Align5.with_params(target_yaw=180.),
    Align5.Aligned: Surge6.with_params(distance=20.),
    Align5.TimedOut: Surface,

    Surge6.Surged: Surface,

    Surface.Surfaced: Stop
}
