#!/usr/bin/env python

from periodic_io import PeriodicIO
import state

# additional imports at bottom to resolve circular dependencies

class PathMarkerAlign(state.State):
    def __call__(self):
        """ Aligns to path marker after crossing gate. """
        # TODO: add logic to align to pathmarker
        return BuoyScan()

class BuoyScan(state.State):
    def __call__(self):
        """ Open loop scans for buoy. """
        # set heading mode to override so we open loop spin
        PeriodicIO.forward = 0
        PeriodicIO.set_override_heading(self.SCAN_SPEED)

        if (PeriodicIO.buoy_position.found):
            return BuoyApproach()

        return self

class BuoyApproach(state.State):
    def __call__(self):
        return BuoyRetreat()

class BuoyRetreat(state.State):
    def __call__(self):
        return BuoyRise()

class BuoyRise(state.State):
    def __call__(self):
        return BuoyForward()

class BuoyForward(state.State):
    def __init__(self):
        from state import Stop
    def __call__(self):
        return state.Stop()

