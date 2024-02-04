#!/usr/bin/env python
from typing import Dict, NamedTuple, Type
from umrsm import StateMachine, State, TransitionMap
import common
import standard_run
# import prequal_strafe
import prequal_turn
import buoy_run
import alignment_test
import spin_test
import rospy
import sys
from periodic_io import PIO
import traceback


# maybe change this to something hacky like getting .transitions from the machine name module?
transition_maps: Dict[str, TransitionMap] = {
    'standard': standard_run.transitions,
    # 'prequal_strafe': prequal_strafe.transitions,
    'prequal_turn': prequal_turn.transitions,
    'buoy': buoy_run.transitions,
    'alignment': alignment_test.transitions,
    'spin': spin_test.transitions,
}


if __name__ == '__main__':
    rospy.init_node('captain')
    machine_name = sys.argv[1]
    machine = StateMachine(
        machine_name,
        transition_maps[machine_name],
        common.Start,
        common.Stop,
    )
    try:
        machine.run()
    except Exception as e:
        print(traceback.format_exc())
        rate = rospy.Rate(50)
        for _ in range(20):
            PIO.reset_target_twist()
            rate.sleep()
