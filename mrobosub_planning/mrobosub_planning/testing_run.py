from mrobosub_planning.common_states import Start, Submerge, Surface, Stop
from mrobosub_planning.mrobosub_planning.testing_states import ComeToSurface, Forward5Seconds
from mrobosub_planning.umrsm import TransitionMap

transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=1.),

    Submerge.Submerged: Forward5Seconds,
    Submerge.TimedOut: Forward5Seconds,

    Forward5Seconds.Reached: ComeToSurface,
    Forward5Seconds.Unreached: ComeToSurface,

    ComeToSurface.Surfaced: Stop,
    ComeToSurface.Failed: Stop,
}