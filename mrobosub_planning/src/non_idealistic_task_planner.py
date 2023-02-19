#!/usr/bin/env python
from periodic_io import PIO
from non_idealistic_states import *
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
    
    def handle(self) -> Outcome:
        return self.Finish()

state_machine = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: AlignGate,
    Submerge.TimedOut: AlignGate,

    AlignGate.Unaligned: AlignGate,
    AlignGate.FoundPathMarker: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.Unreached: ApproachGate,
    ApproachGate.FoundPathMarker: AlignPathMarker,
    ApproachGate.TimedOut: Scan,

    Scan.NotFound: Scan,
    Scan.FoundPathMarker: AlignPathMarker,
    Scan.TimedOut: FallBackTurn,

    FallBackTurn.Unaligned: FallbackTurn,
    FallBackTurn.Aligned: ApproachBuoy,

    AlignPathMarker.Unaligned: AlignPathMarker,
    AlignPathMarker.Aligned: ApproachBuoy,

    ApproachBuoy.Unreached: ApproachBuoy,
    ApproachBuoy.Reached: Center,

    Center.Uncentered: Center,
    Center.Centered: FallBack,

    FallBack.Unaligned: FallBack,
    FallBack.Aligned: Ascend,

    Ascend.Unreached: Ascend,
    Ascend.Reached: Pass,

    Pass.Incomplete: Pass,
    Pass.Complete: Surface,

    Surface.Submerged: Surface,
    Surface.Surfaced: Stop     
}

def main():
    rospy.init_node('task_planner')
    machine = StateMachine(
        'competition', 
        state_machine,
        Start, Surface
    )
    machine.run()

if __name__ == '__main__':
    main()

