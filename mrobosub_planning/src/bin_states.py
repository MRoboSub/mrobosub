from umrsm import State
from abstract_states import TimedState
from periodic_io import PIO, Gbl
from mrobosub_msgs.srv import ObjectPositionResponse
from typing import NamedTuple, Union
import math 

surge_speed = 0.2 #max surge speed
yaw_factor = 0.004
timeout = 40
max_pixel_dist = 500.0 #maximum pixel distance we could see sqrt(maxPixelX**2+maxPixelY**2)
centered_pixel_dist_thresh = 50 #threshold distance in pixels within which we say we have centered appropriatclass CenterToBinFromFar(TimedState):

class CenterToBinFromFar(TimedState):
    class TimedOut(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    timeout: float = 10.0

    def __init__(self, prev_outcome):
        super().__init__(prev_outcome)
        self.dist_to_bin  = max_pixel_dist
        self.angle_to_bin = PIO.Pose.yaw


    def handle_if_not_timedout(self) -> Union[None, Reached]:
        bin_camera_position = PIO.query_BinCamPos() #TODO: make the PIO func return a tuple of x, y camera pos as pixels and bool of if found center for pixels is (0,0)
        if bin_camera_position and bin_camera_position.found: 
            self.angle_to_bin = math.atan2(bin_camera_position.y, bin_camera_position.x) 
            self.dist_to_bin = math.sqrt(bin_camera_position.y**2 + bin_camera_position.x**2)
            PIO.set_target_twist_surge(surge_speed*(self.dist_to_bin/max_pixel_dist)) #set surge speed decreases as closer to centered
            # Use setpoint for yaw angle
            PIO.set_target_twist_yaw(self.angle_to_bin * yaw_factor)
        
        
        

        if self.dist_to_bin < centered_pixel_dist_thresh:
            PIO.set_target_twist_surge(0)
            PIO.set_target_twist_yaw(PIO.Pose.yaw)
            return self.Reached()
        else:
            return None
        

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_yaw(PIO.Pose.yaw)
        return self.TimedOut()
        
# class CenterToBInWhenClose(TimedState):
#     NotReached = Outcome.make('NotReached')
#     TimedOut = Outcome.make('TimedOut')
#     Reached = Outcome.make('Reached')

#     def initialize(self):
        