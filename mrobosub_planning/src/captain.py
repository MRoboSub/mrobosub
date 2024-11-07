#!/usr/bin/env python
from typing import Dict, NamedTuple, Optional, Type
from importlib import import_module
import inspect
from umrsm import StateMachine, State, TransitionMap
import common_states
import standard_run
import re

# import prequal_strafe
import prequal_turn
import buoy_transitions
import bin_transitions
import qual_transitions
import pathmarker_test
import alignment_test
import spin_test
import path_to_bin_transitions
import rospy
import sys
from periodic_io import PIO
import traceback


# maybe change this to something hacky like getting .transitions from the machine name module?
transition_maps: Dict[str, TransitionMap] = {
    "standard": standard_run.transitions,
    "bin_test": bin_transitions.transitions,
    "prequal_turn": prequal_turn.transitions,
    "buoy": buoy_transitions.transitions,
    "alignment": alignment_test.transitions,
    "spin": spin_test.transitions,
    "pathmarker_test": pathmarker_test.transitions,
    "qual": qual_transitions.transitions,
    "path_to_bin": path_to_bin_transitions.transitions,
}


def state_class_from_str(full_state: str, transitions: TransitionMap) -> Optional[Type[State]]:
    """
    Find the associated class object from a given state name.

    Arguments:
        full_state (str): The state passed into roslaunch. Could be in the format `state` or `module.state`.
        transitions (TransitionMap): The transition map from the associated machine name. 
    
    Returns:
        The class object for the state or a ValueError if there is an error finding the state.
    """
    # A NamedTuple is a class made inside of a State. 
    # This helper function finds the state name from a NamedTuple in the transition map. E.g. NamedTuple(StartState.Complete) would return "StartState"
    def named_tuple_to_state_str(named_tuple: Type[NamedTuple]) -> str:
        return named_tuple.__qualname__.split('.')[0]
    
    # Helper function to turn a NamedTuple into its wrapping State. E.g. NamedTuple(StartState.Complete) would return State(StartState)
    def named_tuple_to_state(named_tuple: Type[NamedTuple]) -> Type[State]:
        module = import_module(named_tuple.__module__)
        return getattr(module, named_tuple_to_state_str(named_tuple))
    
    # Ensure that the full_state passed in is either "module.state" or "state". Can have at most one '.'.
    if full_state.count('.') > 1:
        raise ValueError(f"{full_state} should have at most one '.'")
        
    # Return the state from the full_state that looks like "module.state" or "state".
    state = full_state.split('.')[1] if '.' in full_state else full_state

    # The format of a transition map entry is NamedTuple: State.
    # We need to find NamedTuples whose State matches the passed in state.
    found_named_tuples = [named_tuple for named_tuple in transitions.keys() if named_tuple_to_state_str(named_tuple) == state]

    # Turn each candidate NamedTuple into a set of their corresponding States.
    unique_found_states = set([named_tuple_to_state(named_tuple) for named_tuple in found_named_tuples])

    # If the state cannot be found, error.
    if len(unique_found_states) == 0:
        raise ValueError(f"Your selected {state=} is not in {unique_found_states=}. Please enter a state in the transition map.")

    # If multiple states have been found, error.
    if len(unique_found_states) > 1:
        raise ValueError(f"{unique_found_states=} has multiples and can't disambiguate which state to start. Include module or check transition map.")
    
    found_state = unique_found_states.pop()

    # If they provide a module, then confirm that this is the state that the user wants by ensuring the found state's module matches.
    if full_state.count('.') == 1:
        module, _ = full_state.split('.')
        if str(found_state.__module__) != module:
            raise ValueError(f"{found_state=} does not have the same module as the {module=} passed in. Ensure you are using the correct state from the transition map.")
    
    return found_state
    

if __name__ == "__main__":
    rospy.init_node("captain")
    
    machine_name = sys.argv[1]

    # Read the state that the user passed in, which is either
    #  1) `module.state`
    #  2) `state`, and the module has to be inferred
    full_state = sys.argv[2]

    starting_state = state_class_from_str(full_state, transition_maps[machine_name])

    print(starting_state)

    if starting_state is None:
        raise ValueError(f"Could not find {full_state} in the transition map")

    # Syntax `roslaunch mrobosub_planning captain.launch machine:=<machine> state:=<state|module.state>`
    
    machine = StateMachine(
        machine_name,
        transition_maps[machine_name],
        starting_state,
        common_states.Stop,
    )
    try:
        machine.run()
    except Exception as e:
        print(traceback.format_exc())
        rate = rospy.Rate(50)
        for _ in range(20):
            PIO.reset_target_twist()
            rate.sleep()