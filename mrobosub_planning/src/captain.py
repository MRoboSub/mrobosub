#!/usr/bin/env python
from typing import Dict, NamedTuple, Type
from importlib import import_module
import inspect
from umrsm import StateMachine, State, TransitionMap
import common_states
import standard_run

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


def state_class_from_str(full_state: str, transitions: TransitionMap):
    """
    Find the associated class object from a given state name.

    Arguments:
        full_state (str): The state passed into roslaunch. Could be in the format `state` or `module.state`.
        transitions (TransitionMap): The transition map from the associated machine name. 
    
    Returns:
        The class object for the state or `None` if no/multiple are found.
    """
    if '.' in full_state:
        # Split the full state into the module.state_name
        module = full_state.split('.')[0]
        state = full_state.split('.')[1]

        # We are given a module, search for all of the classes in the module
        # Note that inspect.getmembers() returns a tuple that looks like (class string name, class object)
        classes = set(inspect.getmembers(import_module(module), inspect.isclass))

        # Search through all of the tuples by the class string name to 
        # find associated class object
        found_class_options = [tup[1] for tup in classes if tup[0] == state]

        # If we find none or multiple, uhh, break
        if len(found_class_options) != 1:
            return None

        found_class = found_class_options[0]
        return found_class
    else:
        # Assuming that no module is entered in the terminal, we have to find which module the state is in
        # set because we want this to be unique
        module_names = set([cls.__module__ for cls in transitions])

        # We need to get each class for all of the modules included in the transitions module
        # Note that inspect.getmembers() returns a tuple that looks like (class string name, class object)
        classes_in_modules = [inspect.getmembers(import_module(mod), inspect.isclass) for mod in module_names]

        # The above list is a list of lists, flatten this into just a list
        classes_in_transition_map = set([subclass for cls in classes_in_modules for subclass in cls])

        # Search through all of the tuples by the class string name to 
        # find associated class object
        found_class_options = [tup[1] for tup in classes_in_transition_map if tup[0] == full_state]
        
        # If we find none or multiple, uhh, break
        if len(found_class_options) != 1:
            return None
        
        found_class = found_class_options[0]
        return found_class


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
        print(f"Could not find {full_state} in the transition map.")
        exit()

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