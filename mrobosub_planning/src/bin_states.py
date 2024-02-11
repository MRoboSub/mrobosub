from umrsm import Outcome, TimedState, State, Param
from periodic_io import PIO, Gbl, angle_error, Glyph
from mrobosub_msgs.srv import ObjectPositionResponse
import math 
from typing import Mapping


class CenterYawGlyph(TimedState):
    NotReached = Outcome.make('NotReached')
    TimedOut = Outcome.make('TimedOut')
    Reached = Outcome.make('Reached')

    surge_speed: Param[float]
    yaw_factor: Param[float]
    timeout: Param[int]
    max_pixel_dist = 500 #
    centered_pixel_dist_thresh = 20 #threshold distance in pixels within which we say we have centered appropriately

    def initialize(self):
        self.surge_speed = 0.2 #max surge speed
        self.yaw_factor = 0.004
        self.timeout = 40
        self.target_heave = PIO.Pose.heave
       

    def handle_if_not_timedout(self):
        
        bin_camera_position = PIO.get_camera_pos() #TODO: make the PIO func return a tuple of x, y camera pos as pixels as center 0,0, found that is bool 
        if bin_camera_position.found: 
            self.angle_to_bin = math.atan2(bin_camera_position.y, bin_camera_position.x) 

        dist_to_bin = math.sqrt(bin_camera_position.y**2 + bin_camera_position.x**2)


        PIO.set_target_twist_surge(self.surge_speed)

        # Use setpoint for yaw angle
        PIO.set_target_twist_yaw(self.angle_to_bin * self.yaw_factor)
        

        
        if dist_to_bin < self.centered_pixel_dist_thresh:
            return self.Reached()
        else:
            return self.NotReached()

    def handle_once_timedout(self) -> None:
        PIO.set_target_twist_surge(0)
