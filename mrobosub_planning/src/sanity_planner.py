#!/usr/bin/env python
from sanity_states import Submerge
from state_machine import StateMachine, State, Outcome
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
        pass
    
    def handle(self)->Outcome:
        return self.Finish()

state_machine = {
    Start.Complete: Submerge,
    Submerge.SubmergeAgain: Submerge,
    Submerge.TimedOut: Stop,
    Submerge.ReachedDepth: Stop,
}

def main():
    rospy.init_node('task_planner')
    machine = StateMachine(
        'sanity', 
        state_machine,
        Start, Stop
    )
    machine.run()

if __name__ == '__main__':
    main()

