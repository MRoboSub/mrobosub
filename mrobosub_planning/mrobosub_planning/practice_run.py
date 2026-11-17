from mrobosub_planning.common_states import Start, Submerge, Surface, Stop
from mrobosub_planning.testing_states import ComeToSurface
from mrobosub_planning.abstract_states import TurnToYaw, ForwardAndWait
from mrobosub_planning.umrsm import TransitionMap
from mrobosub_planning.testing_states import Forward5Seconds

transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=10),
    
    Submerge.Submerged: Forward5Seconds,
    Submerge.TimedOut:  Forward5Seconds,

    Forward5Seconds.Reached:   ComeToSurface,
    Forward5Seconds.Unreached: ComeToSurface,

    ComeToSurface.Surfaced: Stop,
    ComeToSurface.Failed: Stop,
}
