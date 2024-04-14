from common_states import Start, Submerge, Surface, Stop
from gate_states import Spin, SpinFinish
from umrsm import TransitionMap

transitions: TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: Spin,
    Submerge.TimedOut: Spin,

    Spin.TimedOut: SpinFinish,

    SpinFinish.Reached: Surface,
    SpinFinish.TimedOut: Surface,

    Surface.Surfaced: Stop
}
