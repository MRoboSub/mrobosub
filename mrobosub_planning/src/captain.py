#!/usr/bin/env python
from umrsm import StateMachine, State
import common
# import standard_run
# import prequal_strafe
import prequal_turn
import rospy
import sys



# maybe change this to something hacky like getting .transitions from the machine name module?
transition_maps = {
    'standard': standard_run.transitions,
    # 'prequal_strafe': prequal_strafe.transitions,
    'prequal_turn': prequal_turn.transitions,
    # ...
}


if __name__ == '__main__':
    rospy.init_node('captain')
    machine_name = sys.argv[1]
    machine = StateMachine(
        machine_name,
        transition_maps[machine_name],
        common.Start, common.Stop
    )
    machine.run()
