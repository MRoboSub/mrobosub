from regex import F

from mrobosub_planning.common_states import Start, Submerge, Surface, Stop
from mrobosub_planning.testing_states import ComeToSurface
from mrobosub_planning.abstract_states import TurnToYaw, ForwardAndWait
from mrobosub_planning.umrsm import TransitionMap
Forward5Seconds = ForwardAndWait
Forward5SecondsBack = ForwardAndWait


transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=0.5),
    
    Submerge.Submerged: Forward5Seconds.with_params(target_heave=0.5, target_surge_time=5., wait_time=5., surge_speed=0.1),
    Submerge.TimedOut: Forward5Seconds.with_params(target_heave=0.5, target_surge_time=5., wait_time=5., surge_speed=0.1),

    Forward5Seconds.Reached: TurnToYaw.with_params(target_yaw=180.0, yaw_threshold=5.0, settle_time=1.0, timeout=10.0),
    Forward5Seconds.Unreached: ComeToSurface,

    TurnToYaw.Reached: Forward5SecondsBack.with_params(target_heave=0.5, target_surge_time=5., wait_time=5., surge_speed=0.1),
    TurnToYaw.TimedOut: ComeToSurface,

    Forward5SecondsBack.Reached: ComeToSurface,
    Forward5SecondsBack.Unreached: ComeToSurface,

    ComeToSurface.Surfaced: Stop,
    ComeToSurface.Failed: Stop,
}