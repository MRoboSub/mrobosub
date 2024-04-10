from umrsm import State
from abstract_states import TimedState
from periodic_io import PIO, Gbl
from mrobosub_msgs.srv import ObjectPositionResponse
from typing import NamedTuple, Union
import math 


class CenterToBinFromFar(TimedState):
    class TimedOut(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    timeout: float = 10.0

    surge_speed = 0.2 #max surge speed
    yaw_factor = 1
    max_pixel_dist = 500.0 #maximum pixel distance we could see sqrt(maxPixelX**2+maxPixelY**2)
    centered_pixel_dist_thresh = 50 #threshold distance in pixels within which we say we have centered appropriatclass CenterToBinFromFar(TimedState):

    def __init__(self, prev_outcome):
        super().__init__(prev_outcome)
        self.dist_to_bin  = self.max_pixel_dist
        self.angle_to_bin = PIO.Pose.yaw


    def handle_if_not_timedout(self) -> Union[None, Reached]:
        bin_camera_position = PIO.query_BinCamPos() 
        #print(f"{bin_camera_position=}")
        if bin_camera_position and bin_camera_position.found: 
            #print(f"{bin_camera_position.x=}")
            self.angle_to_bin = math.atan2(bin_camera_position.y, bin_camera_position.x) 
            #print(f"{self.angle_to_bin=}")
            self.angle_to_bin  = math.degrees(-self.angle_to_bin  + math.pi/2)
            self.dist_to_bin = math.sqrt(bin_camera_position.y**2 + bin_camera_position.x**2)
            PIO.set_target_twist_surge(self.surge_speed*(self.dist_to_bin/self.max_pixel_dist)) #set surge speed decreases as closer to centered
            # Use setpoint for yaw angle
            print(f"{self.angle_to_bin=}")
            #print(f"{PIO.Pose.yaw=}")
            #print(f"{self.yaw_factor=}")
            PIO.set_target_pose_yaw((PIO.Pose.yaw % 360 + self.angle_to_bin % 360 * self.yaw_factor) % 360) ## adjust yaw factor in pool testing
        
        if self.dist_to_bin < self.centered_pixel_dist_thresh:
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
        