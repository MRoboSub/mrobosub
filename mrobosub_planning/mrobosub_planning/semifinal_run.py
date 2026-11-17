from mrobosub_planning.common_states import  Start, Submerge, Surface, Stop, ComeUp, Turn180_2, TimeoutFor30
from mrobosub_planning.testing_states import ComeToSurface, Forward5Seconds
from mrobosub_planning.abstract_states import AlignPathmarker, TurnToYaw, ForwardAndWait, CenterOnPathmarker
from mrobosub_planning.umrsm import TransitionMap
from mrobosub_planning.semifinal_states import FindPathmarker, NavigateSlalom, TurnToGate, AlignGate, SemiAlignPathmarker, SemiCenterOnPathmarker, GoToOctagon, TurnTurn

AlignGate2 = AlignGate
Forward5Seconds2 = Forward5Seconds

transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=20),
 
    Submerge.Submerged: TurnToGate,
    Submerge.TimedOut: TurnToGate,

    TurnToGate.Reached: AlignGate,
    TurnToGate.TimedOut: AlignGate,

    AlignGate.Aligned: Forward5Seconds,
    AlignGate.TimedOut: Forward5Seconds,

    Forward5Seconds.Reached: AlignGate2,
    Forward5Seconds.Unreached: AlignGate2,

    AlignGate2.Aligned: Forward5Seconds2,
    AlignGate2.TimedOut: Forward5Seconds2,

    Forward5Seconds2.Reached: TurnTurn,
    Forward5Seconds2.Unreached: TurnTurn,

    TurnTurn.FinishTurn: FindPathmarker,
    TurnTurn.TimedOut: FindPathmarker,

    FindPathmarker.Found: SemiCenterOnPathmarker,
    FindPathmarker.TimedOut: SemiCenterOnPathmarker,

    SemiCenterOnPathmarker.Centered: SemiAlignPathmarker,
    SemiCenterOnPathmarker.TimedOut: SemiAlignPathmarker,

    SemiAlignPathmarker.Aligned: NavigateSlalom,
    SemiAlignPathmarker.TimedOut: NavigateSlalom,

    NavigateSlalom.Navigated: GoToOctagon,
    NavigateSlalom.TimedOut: GoToOctagon,

    GoToOctagon.Reached: ComeToSurface,
    GoToOctagon.TimedOut: ComeToSurface,

    ComeToSurface.Surfaced: Stop,
    ComeToSurface.Failed: Stop,
}