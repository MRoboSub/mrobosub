from common_states import Start, Submerge, Surface, Stop
from gate_states import AlignGate, AlignPathmarker, ApproachGate, ApproachGateImage
from umrsm import TransitionMap

transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.SeenGateImage: ApproachGate, # do not use gate images for qualification
    ApproachGate.TimedOut: Surface,

    Surface.Surfaced: Stop,
}