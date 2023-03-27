#!/usr/bin/env python
from umrsm import StateMachine, State
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
transition_maps = {
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
        common.Start, common.Stop
    )
    try:
        machine.run()
    except Exception as e:
        print(traceback.format_exc())
        rate = rospy.Rate(50)
        for _ in range(20):
            PIO.set_target_twist_heave(0)
            PIO.set_target_twist_yaw(0)
            PIO.set_target_twist_surge(0)
            PIO.set_target_twist_roll(0)
            PIO.set_target_twist_sway(0)
            rate.sleep()