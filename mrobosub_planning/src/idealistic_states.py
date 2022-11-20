from state_machine import Outcome, State, Param
from periodic_io import PIO
from mrobosub_control.msg import HeadingRequest
import rospy
from time import *
import os

#All the states start from here

class PathMarkerAlign(State):
    GoBuoyScan = Outcome.make("BuoyScan")

    def handle(self)->Outcome:
        """ Aligns to path marker after crossing gate. """
        # TODO: add logic to align to pathmarker
        return self.GoBuoyScan()


class BuoyScan(State):
    BuoyScanCont = Outcome.make("BuoyScanCont")
    BuoyApproach = Outcome.make("BuoyApproach")

    def handle(self)->Outcome:
        """ Open loop scans for buoy. """
        # set heading mode to override so we open loop spin
        PIO.forward = 0
        PIO.set_override_heading(self.SCAN_SPEED)

        if (PIO.buoy_position.found):
            return self.BuoyApproach()

        return self.BuoyScanCont()


class BuoyApproach(State):
    BuoyRetreat = Outcome.make("BuoyRetreat")

    def handle(self)->Outcome:
        return self.BuoyRetreat()


class BuoyRetreat(State):
    BuoyRise = Outcome.make("BuoyRise")

    def handle(self)->Outcome:
        return self.BuoyRise()


class BuoyRise(State):
    BuoyForward = Outcome.make("BuoyForward")

    def handle(self)->Outcome:
        return self.BuoyForward()


class BuoyForward(State):
    DoStop = Outcome.make("Outcome")
    
    def handle(self)->Outcome:
        return self.DoStop()


class Scan(State):
    SCAN_SPEED: Param[float]

    ScanAgain = Outcome.make("ScanAgain")
    ApproachGate = Outcome.make("ApproachGate")

    def handle(self)->Outcome:
        # set heading mode to override so we open loop spin
        PIO.forward = 0
        PIO.set_override_heading(self.SCAN_SPEED)

        if (PIO.gate_position.found):
            return self.ApproachGate()

        return self.ScanAgain()


class ApproachGate(State):
    """
    APPROACH_GATE: move towards the gate while keeping it centered until the bootlegger image is seen.
    """
    APPROACH_SCAN_THLD: Param[float]
    APPROACH_IMG_THLD : Param[float]
    HEADING_THLD: Param[float]

    GoSide = Outcome.make("GoSide")
    GoScan = Outcome.make("GoScan")
    GoGateCont = Outcome.make("GoGateCont")

    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()

    def handle(self)->Outcome:
        if PIO.bootlegger_position.found:
            # if we see the image, regardless of whether we see the gate, transition State.
            # TODO: maybe wait until we're closer to the gate/have higher confidence in the image to transition States
            return self.GoSide()

        if PIO.gate_position.found:
            angle_to_gate = 0.5 * PIO.gate_obj.x_diff * self.ZED_FOV  # in degrees
            # TODO: change x_diff in mrobosub_vision/ml_node to get rid of superfluous 1/2

            PIO.set_absolute_heading(
                PIO.current_heading + angle_to_gate
            )

            # make fwd speed > 0 only if we are centered enough
            fwd_speed = 0
            if PIO.heading_within_threshold(self.HEADING_THLD):
                fwd_speed = self.FWD_SPEED # changes forward whip speed

            PIO.forward = fwd_speed
        else:
            if rospy.get_time() - self.start_time > self.APPROACH_IMG_THLD:
                return self.GoSide()
            elif rospy.get_time() - self.start_time > self.APPROACH_SCAN_THLD:
                return self.GoScan()
        
        return self.GoGateCont()


class ApproachSide(State):
    GoCross = Outcome.make("GoCross")
    SideAgain = Outcome.make("SideAgain")

    def initialize(self, prev_outcome: Outcome) -> None:
        self.approach_start = rospy.get_time()

    def handle(self) -> Outcome:
        """
        APPROACH_IMG State: move towards the bootlegger img while keeping it centered until it's out of view.
        """
        angle_to_img = 0.5 * PIO.bootlegger_obj.x_diff * self.ZED_FOV
        # TODO: change x_diff in mrobosub_vision/ml_node to get rid of superfluous 1/2

        # TODO: add contingency in case we never see the image (continue centering on the gate)
        PIO.set_absolute_heading(
            PIO.current_heading + angle_to_img
        )

        # make fwd speed > 0 only if we are centered enough
        fwd_speed = 0
        if PIO.heading_within_threshold(self.HEADING_THLD):
            fwd_speed = self.FWD_SPEED #changes forward whip speed

        if not PIO.bootlegger_obj.found:
            #TODO: if gate is seen when bootlegger is not seen, go back to approach gate
            # transition State
            return self.GoCross()

        PIO.forward = fwd_speed

        return self.SideAgain()


class Cross(State):
    FWD_SPEED: Param[float]
    MOVE_TIME: Param[float]

    TimedOut = Outcome.make('TimedOut')
    CrossAgain = Outcome.make("CrossAgain")

    def initialize(self, prev_outcome: Outcome) -> None:
        self.cross_start = rospy.get_time()

    def handle(self)->Outcome:
        if (rospy.get_time() - self.cross_start) >= self.MOVE_TIME:
            return self.TimedOut()

        PIO.forward = self.FWD_SPEED
        PIO.heading_mode = HeadingRequest.ABSOLUTE
        PIO.heading_value = PIO.current_heading

        return self.CrossAgain()
