from states import *


# TODO: find start and stop state
compeition_transition_map = {
    Start.Complete: Submerge,
    Submerge.SubmergeAgain: Submerge,
    Submerge.GoCrossGate: CrossGate,
    CrossGate.CrossingContinue: CrossGate,
    CrossGate.CrossingDone: Spin,
    Spin.SpinContinue: Spin,
    Spin.SpinReach: GotoBuoy,
    GotoBuoy.GotoBuoyContinue: GotoBuoy,
    GotoBuoy.GoSurface: Surface,
    Surface.SurfaceUp: Surface,
}

buoy_transition_map = {
    PathMarkerAlign.GoBuoyScan: BuoyScan,
    BuoyScan.BuoyScanCont: BuoyScan,
    BuoyScan.BuoyApproach: BuoyApproach,
    BuoyApproach.BuoyRetreat: BuoyRetreat,
    BuoyRetreat.BuoyRise: BuoyRise,
    BuoyRise.BuoyForward: BuoyForward,
    BuoyForward.DoStop: Stop,
}

gate_transition_map = {
    Scan.ScanAgain: Scan,
    Scan.ApproachGate: ApproachGate,
    ApproachGate.GoScan: Scan,
    ApproachGate.GoSide: ApproachSide,
    ApproachGate.GoGateCont: ApproachGate,
    ApproachSide.SideAgain: ApproachSide,
    ApproachSide.GoCross: Cross,
    Cross.CrossAgain: Cross,
    Cross.GoPathMarker: PathMarkerAlign,
    Cross.DoStop: Stop,
}