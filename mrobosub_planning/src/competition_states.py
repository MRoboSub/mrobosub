import rospy
from state_machine import *
from periodic_io import PIO

class Submerge(State):
    submerge_depth: Param[float]
    timeout_time: Param[int]

    TimedOut = Outcome.make('TimedOut')
    ReachedDepth = Outcome.make('ReachedDepth')
    SubmergeAgain = Outcome.make("SubmergeAgain")
    
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()

    def handle(self) -> Outcome:
        """ Submerges to target depth """
        PIO.target_depth = self.submerge_depth
        
        print(PIO.current_depth - self.submerge_depth)
        
        # TODO: change depth logic
        if (PIO.current_depth <= self.submerge_depth + 5):
            return self.ReachedDepth()
        elif (rospy.get_time() - self.start_time > self.timeout_time):
            return self.TimedOut()
        else:
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
    fallback_heading: Param[float]

    SpinContinue = Outcome.make("SpinContinue")
    SpinReach = Outcome.make("SpinReach", heading = fallback_heading)

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
    forward_speed : Param[float]#pwr
    heading_thld : Param[float]#deg
    timeout : Param[float]#sec
    zed_fov: Param[float]

    GotoBuoyContinue = Outcome.make("GotoBuoyContinue", start_heading = 0)
    TimeOut = Outcome.make("TimeOut")

    def initialize(self, prev_outcome) -> None:
        self.target_heading = prev_outcome.start_heading
        self.start_time = rospy.get_time()

    def handle(self) -> Outcome:
        if PIO.gun_position.found:
            angle_to_gate = 0.5 * PIO.gun_position.x_diff * self.zed_fov  # in degrees
            self.target_heading = PIO.current_heading + angle_to_gate

            # if centered enough, continue moving
            if PIO.heading_within_threshold(self.forward_speed):
                PIO.forward = self.forward_speed
            else:
                PIO.forward = 0

        else:
            PIO.forward = self.forward_speed

        PIO.set_absolute_heading(self.target_heading)

        if rospy.get_time() - self.start_time < self.timeout:
            return self.GotoBuoyContinue(start_heading = self.current_heading)
        else:
            return self.TimeOut()


class Surface(State):
    SurfaceUp = Outcome.make("SurfaceUp")
    
    def handle(self)->Outcome:
        print("surface call")
        PIO.target_depth = 0
        return self.SurfaceUp()
        
