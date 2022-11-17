from state_machine import Outcome, State, Param
from periodic_io import PIO
from mrobosub_control.msg import HeadingRequest
import rospy
from time import *
import os


class RunMode:
    FULL = 1
    GATE = 2
    BUOY = 3


run_mode = os.getenv('SUB_RUN_MODE', default='FULL')
run_mode = getattr(RunMode, run_mode, RunMode.FULL)


#All the states start from here

class Submerge(State):
    SUBMERGE_DEPTH: Param[float]

    GoCrossGate = Outcome.make("GoCrossGate")
    SubmergeAgain = Outcome.make("SubmergeAgain")
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()

    def handle(self)->Outcome:
        # print("submerge call")
        """ Submerges to target depth """
        PIO.target_depth = self.SUBMERGE_DEPTH
        
        print(PIO.current_depth - self.SUBMERGE_DEPTH)
        # Move to gate or buoy scan task once target depth reached
        if PIO.current_depth <= self.SUBMERGE_DEPTH + 5 or time() - self.start_time > 10:
            return self.GoCrossGate()

        return self.SubmergeAgain()


class CrossGate(State):
    CrossingContinue = Outcome.make("CrossingContinue")
    CrossingDone = Outcome.make("CrossingDone")

    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self)-> Outcome:
        time = rospy.get_time()
        if 7 < time - self.start_time < 11:
            PIO.roll = 1700
        if 11 < time - self.start_time < 13 and abs(PIO.current_roll) > 45:
            PIO.roll = 1700
        if time -  self.start_time < 15:
            PIO.forward = 1350
        else:
            PIO.forward = 1500
            return self.CrossingDone()
        return self.CrossingContinue()


class Spin(State):
    FALLBACK_HEADING: Param[float]

    SpinContinue = Outcome.make("SpinContinue")
    SpinReach = Outcome.make("SpinReach", heading = FALLBACK_HEADING)

    @staticmethod
    def angle_error(setpoint, state):
        """
        Computes the wrapped error between two angles
        A positive error indicates that the setpoint
        is clockwise of the state
        """
        error = ((setpoint%360) - (state%360)) % 360
        if(error > 180): error -= 360
        return error
    
    @classmethod
    def angle_error_abs(cls, setpoint, state):
        return abs(cls.angle_error(setpoint, state))
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
        self.start_heading = PIO.current_heading
        self.num_spins = 0
        self.near_heading = True

    def handle(self) -> Outcome:
        print(self.angle_error_abs(self.start_heading, PIO.current_heading))

        if self.angle_error_abs(self.start_heading, PIO.current_heading) <= 2 and not self.near_heading:
            self.num_spins += 1
            self.near_heading = True
        elif self.angle_error_abs(self.start_heading, PIO.current_heading) >= 10:
            self.near_heading = False

        if self.num_spins >= 3:
            return self.SpinReach()
           # return GotoBuoy(self.FALLBACK_HEADING)

        if PIO.gun_position.found:
            return self.SpinReach(heading = PIO.current_heading)
          #  return GotoBuoy(PeriodicIO.current_heading)

        PIO.set_override_heading(1550)
        return self.SpinContinue()


class GotoBuoy(State):
    FORWARD_SPEED : Param[float]#pwr
    HEADING_THLD : Param[float]#deg
    TIMEOUT : Param[float]#sec

    GotoBuoyContinue = Outcome.make("GotoBuoyContinue", start_heading = 0)
    GoSurface = Outcome.make("GoSurface")

    def initilize(self, prev_outcome) -> None:
        self.target_heading = prev_outcome.start_heading
        self.start_time = rospy.get_time()

    def handle(self) -> Outcome:
        if PIO.gun_position.found:
            angle_to_gate = 0.5 * PIO.gun_position.x_diff * self.ZED_FOV  # in degrees
            self.target_heading = PIO.current_heading + angle_to_gate

            # if centered enough, continue moving
            if PIO.heading_within_threshold(self.HEADING_THLD):
                PIO.forward = self.FORWARD_SPEED
            else:
                PIO.forward = 0

        else:
            PIO.forward = self.FORWARD_SPEED

        PIO.set_absolute_heading(self.target_heading)

        if time() - self.start_time < self.TIMEOUT:
            return self.GotoBuoyContinue(start_heading = self.current_heading)
        else:
            return self.GoSurface()


class Surface(State):
    SurfaceUp = Outcome.make("SurfaceUp")
    
    def handle(self)->Outcome:
        print("surface call")
        PIO.target_depth = 0
        return self.SurfaceUp()
        # TODO: make an actual end state here


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

    GoPathMarker = Outcome.make("GoPathMarker")
    DoStop = Outcome.make("DoStop")
    CrossAgain = Outcome.make("CrossAgain")

    def initialize(self, prev_outcome: Outcome) -> None:
        self.cross_start = rospy.get_time()

    def handle(self)->Outcome:
        if (rospy.get_time() - self.cross_start) >= self.MOVE_TIME:
            if run_mode == RunMode.FULL:
                return self.GoPathMarker()
            else:
                return self.DoStop()

        PIO.forward = self.FWD_SPEED
        PIO.heading_mode = HeadingRequest.ABSOLUTE
        PIO.heading_value = PIO.current_heading

        return self.CrossAgain()


class Start(State):
    Complete = Outcome.make("Complete")
    
    def initilize(self, prev_outcome: Outcome) -> None:
        print("init state call")
    
    def handle(self)->Outcome:
        return self.Complete()


class Stop(State):
    Finish = Outcome.make("Finish")

    def initilize(self, prev_outcome: Outcome) -> None:
        PIO.forward = 0
        PIO.lateral = 0
        PIO.target_depth = 0
        PIO.set_override_heading(0)
    
    def handle(self)->Outcome:
        return self.Finish()
