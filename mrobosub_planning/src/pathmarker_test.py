from gate_states import AlignSlalomPathmarker, ApproachGate
from buoy_states import ApproachBuoyOpen
from common_states import Start, Submerge, Surface, Stop
from abstract_states import CenterOnPathmarker
from umrsm import Outcome, TransitionMap


class CenterOnPathmarkerImpl(CenterOnPathmarker):
    class Complete(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    timeout: float = 30.0

    def handle_aligned(self) -> Outcome:
        return self.Complete()

    def handle_once_timedout(self) -> Outcome:
        return self.TimedOut()


transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=0.4),

    Submerge.Submerged: CenterOnPathmarkerImpl,
    Submerge.TimedOut: CenterOnPathmarkerImpl,

    CenterOnPathmarkerImpl.Complete: Surface,
    CenterOnPathmarkerImpl.TimedOut: Surface,

    Surface.Surfaced: Stop,
}
