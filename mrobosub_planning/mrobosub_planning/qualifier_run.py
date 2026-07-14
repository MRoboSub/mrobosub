from mrobosub_planning.common_states import Forward10, Start, Submerge, Surface, Stop, ComeUp, ReturnHome, Turn180_2, TimeoutFor30, RevertCoinFlip
from mrobosub_planning.testing_states import ComeToSurface, Forward10Seconds, Turn180
from mrobosub_planning.abstract_states import TurnToYaw, ForwardAndWait
from mrobosub_planning.umrsm import TransitionMap

Forward10SecondsBack = Forward10Seconds

transitions: TransitionMap = {
    Start.Complete: Submerge,
    
    Submerge.Submerged: Forward10, #RevertCoinFlip,
    Submerge.TimedOut: Forward10, #evertCoinFlip, 

    # RevertCoinFlip.Reverted: Forward10,
    # RevertCoinFlip.TimedOut: Forward10,

    Forward10.ReachedGate: Turn180_2,
    Forward10.TimedOut: Turn180_2,

    Turn180_2.ReachedGate: ReturnHome,
    Turn180_2.TimedOut: ReturnHome,

    ReturnHome.ReachedGate: ComeUp,
    ReturnHome.TimedOut: ComeUp,

#    Turn180.Reached: Forward10SecondsBack,
#    Turn180.TimedOut: Forward10SecondsBack,

#    Forward10SecondsBack.Reached: ComeToSurface,
#    Forward10SecondsBack.Unreached: ComeToSurface,

    ComeUp.ReachedGate: Stop,
    ComeUp.TimedOut: Stop,

#     Start.Complete: Submerge,
    
#     Submerge.Submerged: Turn180_2,
#     Submerge.TimedOut: Turn180_2,

#     # Forward10.ReachedGate: Turn180_2,
#     # Forward10.TimedOut: Turn180_2,

#     Turn180_2.ReachedGate: ComeUp,
#     Turn180_2.TimedOut: ComeUp,

#     # ReturnHome.ReachedGate: ComeUp,
#     # ReturnHome.TimedOut: ComeUp,

# #    Turn180.Reached: Forward10SecondsBack,
# #    Turn180.TimedOut: Forward10SecondsBack,

# #    Forward10SecondsBack.Reached: ComeToSurface,
# #    Forward10SecondsBack.Unreached: ComeToSurface,

#     ComeUp.ReachedGate: Stop,
#     ComeUp.TimedOut: Stop,
}
