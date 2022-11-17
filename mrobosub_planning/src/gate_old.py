#!/usr/bin/env python

import time
from mrobosub_control.msg import HeadingRequest
from periodic_io import PeriodicIO
from util.angles import *
from state import State, run_mode, RunMode, Stop

# additional imports at bottom to resolve circular dependencies

class Scan(State):
    SCAN_SPEED = 70

    def __call__(self):
        # set heading mode to override so we open loop spin
        PeriodicIO.forward = 0
        PeriodicIO.set_override_heading(self.SCAN_SPEED)

        if (PeriodicIO.gate_position.found):
            return ApproachGate()

        return self

class ApproachGate(State):
    """
    APPROACH_GATE: move towards the gate while keeping it centered until the bootlegger image is seen.
    """
    APPROACH_SCAN_THLD = 0.5
    APPROACH_IMG_THLD = 5
    HEADING_THLD = 30

    def __init__(self):
        self.start_time = time.time()

    def __call__(self):
        if PeriodicIO.bootlegger_position.found:
            # if we see the image, regardless of whether we see the gate, transition State.
            # TODO: maybe wait until we're closer to the gate/have higher confidence in the image to transition States
            return ApproachSide()

        if PeriodicIO.gate_position.found:
            angle_to_gate = 0.5 * PeriodicIO.gate_obj.x_diff * self.ZED_FOV  # in degrees
            # TODO: change x_diff in mrobosub_vision/ml_node to get rid of superfluous 1/2

            PeriodicIO.set_absolute_heading(
                PeriodicIO.current_heading + angle_to_gate
            )

            # make fwd speed > 0 only if we are centered enough
            fwd_speed = 0
            if PeriodicIO.heading_within_threshold(self.HEADING_THLD):
                fwd_speed = self.FWD_SPEED # changes forward whip speed

            PeriodicIO.forward = fwd_speed
        else:
            if time.time() - self.start_time > self.APPROACH_IMG_THLD:
                return ApproachSide()
            elif time.time() - self.start_time > self.APPROACH_SCAN_THLD:
                return Scan()
        
        return self

class ApproachSide(State):
    def __init__(self):
        self.approach_start = time.time()

    def __call__(self):
        """
        APPROACH_IMG State: move towards the bootlegger img while keeping it centered until it's out of view.
        """
        angle_to_img = 0.5 * PeriodicIO.bootlegger_obj.x_diff * self.ZED_FOV
        # TODO: change x_diff in mrobosub_vision/ml_node to get rid of superfluous 1/2

        # TODO: add contingency in case we never see the image (continue centering on the gate)
        PeriodicIO.set_absolute_heading(
            PeriodicIO.current_heading + angle_to_img
        )

        # make fwd speed > 0 only if we are centered enough
        fwd_speed = 0
        if PeriodicIO.heading_within_threshold(self.HEADING_THLD):
            fwd_speed = self.FWD_SPEED #changes forward whip speed

        if not PeriodicIO.bootlegger_obj.found:
            #TODO: if gate is seen when bootlegger is not seen, go back to approach gate
            # transition State
            return Cross()

        PeriodicIO.forward = fwd_speed

        return self

class Cross(State):
    FWD_SPEED = 100
    MOVE_TIME = 0.5

    def __init__(self):
        self.cross_start = time.time()

    def __call__(self):
        if (time.time() - self.cross_start) >= self.MOVE_TIME:
            if run_mode == RunMode.FULL:
                return buoy.PathMarkerAlign()
            else:
                return Stop()

        PeriodicIO.forward = self.FWD_SPEED
        PeriodicIO.heading_mode = HeadingRequest.ABSOLUTE
        PeriodicIO.heading_value = PeriodicIO.current_heading

        return self

import buoy

