#!/usr/bin/env python
from competition_states import *
from idealistic_states import *
from state_machine import StateMachine, State
import rospy

class Start(State):
    Complete = Outcome.make("Complete")
    
    def initialize(self, prev_outcome: Outcome) -> None:
        print("init state call")
    
    def handle(self)->Outcome:
        return self.Complete()

class Stop(State):
    Finish = Outcome.make("Finish")

    def initialize(self, prev_outcome: Outcome) -> None:
        PIO.forward = 0
        PIO.lateral = 0
        PIO.target_depth = 0
        PIO.set_override_heading(0)
    
    def handle(self)->Outcome:
        return self.Finish()

compeition_transition_map = {
    Start.Complete: Submerge,
    Submerge.SubmergeAgain: Submerge,
    Submerge.TimedOut: CrossGate,
    Submerge.ReachedDepth: CrossGate,
    CrossGate.CrossingContinue: CrossGate,
    CrossGate.CrossingDone: Spin,
    Spin.SpinContinue: Spin,
    Spin.SpinReach: GotoBuoy,
    GotoBuoy.GotoBuoyContinue: GotoBuoy,
    GotoBuoy.TimeOut: Surface,
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
    Cross.TimedOut: Stop,
}

def main():
    rospy.init_node('task_planner')
    machine = StateMachine(
        'competition', 
        compeition_transition_map,
        Start, Surface
    )
    machine.run()

if __name__ == '__main__':
    main()

