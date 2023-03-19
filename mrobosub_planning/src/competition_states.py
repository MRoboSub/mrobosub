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
        PIO.set_target_pose_heave(self.submerge_depth)
        

        print(PIO.get_pose().heave - self.submerge_depth)
        
        # TODO: change depth logic
        if (PIO.get_pose().heave <= self.submerge_depth + 5):
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
            self.target_heading = PIO.get_pose().yaw + angle_to_gate

            # if centered enough, continue moving
            if PIO.heading_within_threshold(self.forward_speed):
                PIO.set_target_twist_surge(self.forward_speed)
            else:
                PIO.set_target_twist_surge(0)

        else:
            PIO.set_target_twist_surge(self.forward_speed)

        PIO.set_target_pose_yaw(self.target_heading)

        if rospy.get_time() - self.start_time < self.timeout:
            return self.GotoBuoyContinue(start_heading = self.current_heading)
        else:
            return self.TimeOut()


class Surface(State):
    SurfaceUp = Outcome.make("SurfaceUp")
    
    def handle(self)->Outcome:
        print("surface call")
        PIO.set_target_pose_heave(0)
        return self.SurfaceUp()
        
