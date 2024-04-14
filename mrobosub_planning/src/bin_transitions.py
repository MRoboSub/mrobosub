from common_states import *
from bin_states import *
from umrsm import TransitionMap


transitions:TransitionMap = {
    Start.Complete: Submerge,

    Submerge.Submerged: CenterToBinFromFar,
    Submerge.TimedOut: CenterToBinFromFar,

    CenterToBinFromFar.Reached: Surface,
    CenterToBinFromFar.TimedOut: Surface,

    Surface.Surfaced: Stop
}
