""" States which are deprecated but may contain useful logic to replicate. """


from umrsm import Outcome, State, Param
from periodic_io import PIO
import rospy

class ApproachGate(State):
    """
    APPROACH_GATE: move towards the gate while keeping it centered until the bootlegger image is seen.
    """
    approach_scan_thld: Param[float]
    approach_img_thld : Param[float]
    heading_thld: Param[float]

    GoSide = Outcome.make("GoSide")
    GoScan = Outcome.make("GoScan")
    GoGateCont = Outcome.make("GoGateCont")

    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()

    def handle(self) -> Outcome:
        # TODO
        if PIO.bootlegger_position.found:
            # if we see the image, regardless of whether we see the gate, transition State.
            # TODO: maybe wait until we're closer to the gate/have higher confidence in the image to transition States
            return self.GoSide()

        if PIO.gate_position.found:
            angle_to_gate = 0.5 * PIO.gate_obj.x_diff * self.zed_fov  # in degrees
            # TODO: change x_diff in mrobosub_vision/ml_node to get rid of superfluous 1/2

            PIO.set_absolute_heading(
                PIO.current_heading + angle_to_gate
            )

            # make fwd speed > 0 only if we are centered enough
            fwd_speed = 0
            if PIO.heading_within_threshold(self.heading_thld):
                fwd_speed = self.fwd_speed # changes forward whip speed

            PIO.set_target_twist_surge(fwd_speed)
        else:
            if rospy.get_time() - self.start_time > self.approach_img_thld:
                return self.GoSide()
            elif rospy.get_time() - self.start_time > self.approach_scan_thld:
                return self.GoScan()
        
        return self.GoGateCont()


class ApproachSide(State):
    heading_thld: Param[float]
    fwd_speed: Param[float]

    GoCross = Outcome.make("GoCross")
    SideAgain = Outcome.make("SideAgain")

    def initialize(self, prev_outcome: Outcome) -> None:
        self.approach_start = rospy.get_time()

    def handle(self) -> Outcome:
        """
        APPROACH_IMG State: move towards the bootlegger img while keeping it centered until it's out of view.
        """
        angle_to_img = 0.5 * PIO.bootlegger_obj.x_diff * self.zed_fov
        # TODO: change x_diff in mrobosub_vision/ml_node to get rid of superfluous 1/2

        # TODO: add contingency in case we never see the image (continue centering on the gate)
        PIO.set_target_pose_yaw(PIO.TargetPose.yaw + angle_to_img)

        # make fwd speed > 0 only if we are centered enough
        fwd_speed = 0
        if PIO.is_yaw_within_threshold(self.heading_thld):
            fwd_speed = self.fwd_speed #changes forward whip speed

        if not PIO.bootlegger_obj.found:
            #TODO: if gate is seen when bootlegger is not seen, go back to approach gate
            # transition State
            return self.GoCross()

        PIO.set_target_pose_heave(fwd_speed)

        return self.SideAgain()

class CrossGate(State):
    CrossingContinue = Outcome.make("CrossingContinue")
    CrossingDone = Outcome.make("CrossingDone")

    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self)-> Outcome:
        time = rospy.get_time()
        if 7 < time - self.start_time < 11:
            PIO.set_target_twist_roll(1700)
        if 11 < time - self.start_time < 13 and abs(PIO.get_pose().roll) > 45:
            PIO.set_target_twist_roll(1700)
        if time - self.start_time < 15:
            PIO.set_target_twist_surge(1350)
        else:
            PIO.set_target_twist_surge(1500)
            return self.CrossingDone()
        return self.CrossingContinue()


class Spin(State):
    fallback_heading: Param[float]

    SpinContinue = Outcome.make("SpinContinue")
    SpinReach = Outcome.make("SpinReach", heading = float)

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
        self.start_heading = PIO.get_pose().yaw
        self.num_spins = 0
        self.near_heading = True

    def handle(self) -> Outcome:
        print(self.angle_error_abs(self.start_heading, PIO.get_pose().yaw))

        if self.angle_error_abs(self.start_heading, PIO.get_pose().yaw) <= 2 and not self.near_heading:
            self.num_spins += 1
            self.near_heading = True
        elif self.angle_error_abs(self.start_heading, PIO.get_pose().yaw) >= 10:
            self.near_heading = False

        if self.num_spins >= 3:
            return self.SpinReach(heading = self.fallback_heading)
           # return GotoBuoy(self.FALLBACK_HEADING)

        if PIO.gun_position.found:
            return self.SpinReach(heading = PIO.get_pose().yaw)
          #  return GotoBuoy(PeriodicIO.current_heading)

        PIO.set_target_twist_yaw(1550)
        return self.SpinContinue()

