from common import *
from bin_states import *


transitions = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: CenterToBinFromFar,
    Submerge.TimedOut: CenterToBinFromFar,

    CenterToBinFromFar.NotReached: CenterToBinFromFar,
    CenterToBinFromFar.Reached: Surface,
    CenterToBinFromFar.TimedOut: Surface,

    Surface.Submerged: Surface,
    Surface.Surfaced: Stop
}
