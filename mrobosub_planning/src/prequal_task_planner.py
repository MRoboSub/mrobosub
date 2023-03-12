#!/usr/bin/env python
from periodic_io import PIO
from prequal_states import *
from state_machine import StateMachine, State, Outcome, TimedState, Param
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

class Submerge(TimedState):
    Unreached = Outcome.make("Unreached")
    Submerged = Outcome.make("Submerged")
    TimedOut = Outcome.make("TimedOut")
    
    target_depth: Param[float]
    timeout: Param[int]
    
    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_heave(self.target_depth)

        if PIO.Pose.heave > self.target_depth:
            return self.Submerged()
        else:
            return self.Unreached()
    

state_machine = {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: AlignGate,

    AlignGate.Unaligned: AlignGate,
    AlignGate.ReachedAngle: ApproachGate,
    AlignGate.TimedOut: ApproachGate,

    ApproachGate.Unreached: ApproachGate,
    ApproachGate.Reached: ApproachMarker,

    ApproachMarker.Unreached: ApproachMarker,
    ApproachMarker.Reached: MoveAroundMarker,

    MoveAroundMarker.Unreached: MoveAroundMarker,
    MoveAroundMarker.Reached: ReturnGate,

    ReturnGate.Unreached: ReturnGate,
    ReturnGate.Reached: Stop,
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
