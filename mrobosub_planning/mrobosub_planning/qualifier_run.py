from mrobosub_planning.common_states import Start, Submerge, Surface, Stop
from mrobosub_planning.testing_states import ComeToSurface, Forward10Seconds, Turn180
from mrobosub_planning.abstract_states import TurnToYaw, ForwardAndWait
from mrobosub_planning.umrsm import TransitionMap

Forward10SecondsBack = Forward10Seconds

transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=10),
    
    Submerge.Submerged: Forward10Seconds,
    Submerge.TimedOut: Forward10Seconds,

    Forward10Seconds.Reached: ComeToSurface,
    Forward10Seconds.Unreached: ComeToSurface,

#    Turn180.Reached: Forward10SecondsBack,
#    Turn180.TimedOut: Forward10SecondsBack,

#    Forward10SecondsBack.Reached: ComeToSurface,
#    Forward10SecondsBack.Unreached: ComeToSurface,

    ComeToSurface.Surfaced: Stop,
    ComeToSurface.Failed: Stop,
}
