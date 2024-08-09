from gate_states import AlignBuoyPathmarker, ApproachGate
from buoy_states import ApproachBuoyOpen
from common_states import Start, Submerge, Surface, Stop
from umrsm import TransitionMap


transitions: TransitionMap = {
    Start.Complete: AlignBuoyPathmarker,#Submerge,

    Submerge.Submerged: AlignBuoyPathmarker,
    Submerge.TimedOut: AlignBuoyPathmarker,

    AlignBuoyPathmarker.NoMeasurements: Surface,
    AlignBuoyPathmarker.AlignedToBuoy: Surface,
    AlignBuoyPathmarker.TimedOut: Surface,

    Surface.Surfaced: Stop
}
