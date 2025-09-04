#!/usr/bin/env python
from typing import Dict, Type
from importlib import import_module
# from umrsm import StateMachine, State, TransitionMap, Outcome
# import mrobosub_planning.common_states
# import mrobosub_planning.standard_run2

# import prequal_strafe
import rclpy
from rclpy.node import Node
import sys
from mrobosub_planning.periodic_io import PIO
import traceback


# maybe change this to something hacky like getting .transitions from the machine name module?
# transition_maps: Dict[str, TransitionMap] = {
#     "standard": standard_run2.transitions,
#     "heave_test": heave_test.transitions,
# }


# def state_class_from_str(full_state: str, transitions: TransitionMap) -> Type[State]:
#     """
#     Find the associated class object from a given state name.

#     Arguments:
#         full_state (str): The state passed into roslaunch. Could be in the format `state` or `module.state`.
#         transitions (TransitionMap): The transition map from the associated machine name. 
    
#     Returns:
#         The class object for the state or a ValueError if there is an error finding the state.
#     """
#     def outcome_to_state_str(outcome: Type[Outcome]) -> str:
#         return outcome.__qualname__.split('.')[0]
    
#     def outcome_to_state(outcome: Type[Outcome]) -> Type[State]:
#         module = import_module(outcome.__module__)
#         return getattr(module, outcome_to_state_str(outcome))
    
#     if full_state.count('.') > 1:
#         raise ValueError(f"{full_state} should have at most one '.'")

#     state = full_state.split('.')[-1]
#     found_outcomes = [outcome for outcome in transitions.keys() if outcome_to_state_str(outcome) == state]
#     unique_found_states = set(outcome_to_state(outcome) for outcome in found_outcomes)
    
#     if '.' in full_state:
#         unique_found_states = set(state for state in unique_found_states if state.__module__ == full_state.split(".")[0])

#     if len(unique_found_states) != 1:
#         raise ValueError(f'{full_state=} does not uniquely describe a state. {unique_found_states=}')
    
#     found_state = unique_found_states.pop()
#     return found_state

def main(args=None):
    rclpy.init(args=args)
    captain_node = PIO(name="captain")
    captain_node.get_logger().info("HERE")

    # Syntax `roslaunch mrobosub_planning captain.launch machine:=<machine> state:=<state|module.state>`
    machine_name = sys.argv[1]
    full_state = sys.argv[2]

    starting_state = state_class_from_str(full_state, transition_maps[machine_name])
    
    machine = StateMachine(
        machine_name,
        transition_maps[machine_name],
        starting_state,
        common_states.Stop,
    )
    try:
        machine.run()
    except Exception as e:
        node.logger.info({traceback.format_exc()})
        rate = node.create_rate(50)
        for _ in range(20):
            PIO.reset_target_twist()
            rate.sleep()

if __name__ == "__main__":
    main()
