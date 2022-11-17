from periodic_io import PeriodicIO
from time import time
import state
from util.angles import angle_error_abs


class CrossGate(state.State):
    def __init__(self):
        self.start_time = time()
    
    def __call__(self):
        if 7 < time() - self.start_time < 11:
            PeriodicIO.roll = 1700
        if 11 < time() - self.start_time < 13 and abs(PeriodicIO.current_roll) > 45:
            PeriodicIO.roll = 1700
        if time() -  self.start_time < 15:
            PeriodicIO.forward = 1350
        else:
            PeriodicIO.forward = 1500
            return Spin()
        return self

class Spin(state.State):
    FALLBACK_HEADING = 45

    def __init__(self):
        self.start_time = time()
        self.start_heading = PeriodicIO.current_heading
        self.num_spins = 0
        self.near_heading = True
        
    def __call__(self):
        print(angle_error_abs(self.start_heading, PeriodicIO.current_heading))

        if angle_error_abs(self.start_heading, PeriodicIO.current_heading) <= 2 and not self.near_heading:
            self.num_spins += 1
            self.near_heading = True
        elif angle_error_abs(self.start_heading, PeriodicIO.current_heading) >= 10:
            self.near_heading = False

        if self.num_spins >= 3:
            return GotoBuoy(self.FALLBACK_HEADING)

        if PeriodicIO.gun_position.found:
            return GotoBuoy(PeriodicIO.current_heading)

        PeriodicIO.set_override_heading(1550)
        return self


class GotoBuoy(state.State):
    FORWARD_SPEED = 1600 #pwr
    HEADING_THLD = 20 #deg
    TIMEOUT = 15 #sec

    def __init__(self, start_heading):
        self.target_heading = start_heading
        self.start_time = time()

    def __call__(self):
        if PeriodicIO.gun_position.found:
            angle_to_gate = 0.5 * PeriodicIO.gun_position.x_diff * self.ZED_FOV  # in degrees
            self.target_heading = PeriodicIO.current_heading + angle_to_gate

            # if centered enough, continue moving
            if PeriodicIO.heading_within_threshold(self.HEADING_THLD):
                PeriodicIO.forward = self.FORWARD_SPEED
            else:
                PeriodicIO.forward = 0

        else:
            PeriodicIO.forward = self.FORWARD_SPEED

        PeriodicIO.set_absolute_heading(self.target_heading)

        if time() - self.start_time < self.TIMEOUT:
            return self
        else:
            return Surface()

class Surface(state.State):
    def __call__(self):
        print("surface call")
        PeriodicIO.target_depth = 0
        return self

