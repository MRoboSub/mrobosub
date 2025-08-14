from common_states import Start, Submerge, Surface, Stop
from gate_states import AlignGate, ApproachGate
from umrsm import TransitionMap

transitions: TransitionMap = {
    Start.Complete: Submerge.with_params(target_heave=1),

    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.ReachedAngle: Submerge.with_params(target_heave=0.2),
    AlignGate.TimedOut: Surface,

    Submerge.Submerged: Surface,
    Submerge.TimedOut: Surface,

    Surface.Surfaced: Stop,
}
