from common import Start, Submerge, Surface, Stop
from gate_task import Spin, SpinFinish
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
