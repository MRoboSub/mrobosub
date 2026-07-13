from mrobosub_planning.common_states import Start, Submerge, Surface, Stop
from mrobosub_planning.testing_states import ComeToSurface, Forward5Seconds, Forward10Seconds
from mrobosub_planning.abstract_states import TurnToYaw
from mrobosub_planning.umrsm import TransitionMap

Forward10SecondsBack = Forward10Seconds.with_params(target_heave=0.5, target_surge_time=10., wait_time=10., surge_speed=0.1)
Forward5SecondsBack = Forward5Seconds.with_params(target_heave=0.5, target_surge_time=5., wait_time=5., surge_speed=0.1)


transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=0.5),
    
    Submerge.Submerged: Forward10Seconds,
    Submerge.TimedOut: Forward10Seconds,

    Forward10Seconds.Reached: TurnToYaw.with_params(target_yaw=180.0, yaw_threshold=5.0, settle_time=1.0, timeout=10.0),
    Forward10Seconds.Unreached: ComeToSurface,

    TurnToYaw.Reached: Forward10SecondsBack,
    TurnToYaw.TimedOut: ComeToSurface,

    Forward10SecondsBack.Reached: ComeToSurface,
    Forward10SecondsBack.Unreached: ComeToSurface,

    ComeToSurface.Surfaced: Stop,
    ComeToSurface.Failed: Stop,
}