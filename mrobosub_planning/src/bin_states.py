from umrsm import State
from abstract_states import TimedState
from periodic_io import PIO, Gbl
from mrobosub_msgs.srv import ObjectPositionResponse
from typing import NamedTuple, Union
import math 
import rospy
from std_msgs.msg import Int32


class ApproachBin(TimedState):
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
        self.dist_to_bin: float  = self.max_pixel_dist
        self.angle_to_bin: float = 0.0


    def handle_if_not_timedout(self) -> Union[None, Reached]:
        bin_camera_position = PIO.query_BinCamPos() 
        #print(f"{bin_camera_position=}")
        if bin_camera_position and bin_camera_position.found:
            x, y = bin_camera_position.x_position, bin_camera_position.y_position
            self.angle_to_bin = math.atan2(y, x) 
            self.angle_to_bin = math.degrees(-self.angle_to_bin + math.pi/2)
            self.dist_to_bin = math.sqrt(y**2 + x**2)
            PIO.set_target_twist_surge(self.surge_speed*(self.dist_to_bin/self.max_pixel_dist)) #set surge speed decreases as closer to centered
            
            # Use setpoint for yaw angle
            PIO.set_target_pose_yaw((PIO.Pose.yaw % 360 + self.angle_to_bin % 360 * self.yaw_factor) % 360) ## adjust yaw factor in pool testing
        
        if self.dist_to_bin < self.centered_pixel_dist_thresh:
            PIO.set_target_twist_surge(0)
            PIO.set_target_twist_yaw(PIO.Pose.yaw) #TODO: why is this twist and above is pose
            return self.Reached()
        else:
            return None
        

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_yaw(PIO.Pose.yaw)
        return self.TimedOut()

        

class CenterOrSpin(NamedTuple): #Used to mark which previous state we came from to allow for dropper code to work
    is_center: bool

class CenterCameraToBin(TimedState):
    class TimedOut(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    timeout: float = 10.0

    surge_and_strafe_speed = 0.2 #max surge/sway speed
    yaw_factor = 1
    max_pixel_dist = 500.0 #maximum pixel distance we could see sqrt(maxPixelX**2+maxPixelY**2)
    centered_pixel_x_y_thresh = 50 #threshold distance in pixels if x is within then no sway and if y is within then no y

    def __init__(self, prev_outcome):
        super().__init__(prev_outcome)
        self.dist_to_bin: float  = self.max_pixel_dist
        self.angle_to_bin: float = 0.0

    def handle_if_not_timedout(self) -> Union[None, Reached]:
        bin_camera_position = PIO.query_BinCamPos() 
        if bin_camera_position and bin_camera_position.found: 
            self.angle_to_bin = math.atan2(bin_camera_position.y, bin_camera_position.x) 

            surge_speed:float = 0.0
            sway_speed:float = 0.0
            
            if bin_camera_position.y - self.centered_pixel_x_y_thresh / 2 > 0:
                surge_speed = self.surge_and_strafe_speed
            elif bin_camera_position.y + self.centered_pixel_x_y_thresh / 2 < 0:
                surge_speed = -1 * self.surge_and_strafe_speed
            if bin_camera_position.x - self.centered_pixel_x_y_thresh / 2 > 0:
                sway_speed = self.surge_and_strafe_speed
            elif bin_camera_position.x + self.centered_pixel_x_y_thresh / 2 < 0:
                sway_speed = -1 * self.surge_and_strafe_speed

            PIO.set_target_twist_surge(surge_speed)
            PIO.set_target_twist_sway(sway_speed)

            if surge_speed == 0 and sway_speed == 0:
                return self.Reached()
            else:
                return None
            
    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_sway(0)
        return self.TimedOut()
    


class CenterLeftDropper(TimedState):
    class TimedOut(NamedTuple):
        pass
    class Reached(CenterOrSpin):
        pass
    timeout: float = 10.0

    surge_and_strafe_speed = 0.2 #max surge/sway speed
    pixel_dropper_offset = 50
    yaw_factor = 1
    max_pixel_dist = 500.0 #maximum pixel distance we could see sqrt(maxPixelX**2+maxPixelY**2)
    centered_pixel_x_y_thresh = 50 #threshold distance in pixels within which we say we have centered appropriatclass CenterToBinFromFar(TimedState):

    def __init__(self, prev_outcome):
        super().__init__(prev_outcome)
        self.dist_to_bin: float  = self.max_pixel_dist
        self.angle_to_bin: float = 0.0

    def handle_if_not_timedout(self) -> Union[None, Reached]:
        bin_camera_position = PIO.query_BinCamPos() 
        if bin_camera_position and bin_camera_position.found: 
            self.angle_to_bin = math.atan2(bin_camera_position.y, bin_camera_position.x) 

            surge_speed:float = 0.0
            sway_speed:float = 0.0
            
            if bin_camera_position.y - self.centered_pixel_x_y_thresh / 2 > 0:
                surge_speed = self.surge_and_strafe_speed
            elif bin_camera_position.y + self.centered_pixel_x_y_thresh / 2 < 0:
                surge_speed = -1 * self.surge_and_strafe_speed
            if bin_camera_position.x - self.centered_pixel_x_y_thresh - self.pixel_dropper_offset / 2 > 0:
                sway_speed = self.surge_and_strafe_speed
            elif bin_camera_position.x + self.centered_pixel_x_y_thresh - self.pixel_dropper_offset/ 2 < 0:
                sway_speed = -1 * self.surge_and_strafe_speed

            PIO.set_target_twist_surge(surge_speed)
            PIO.set_target_twist_sway(sway_speed)

            if surge_speed == 0 and sway_speed == 0:
                return self.Reached(True)
            else:
                return None
            
    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_sway(0)
        return self.TimedOut()

        
            
class Descend(TimedState):
    class Reached(NamedTuple): pass
    class TimedOut(NamedTuple): pass
    timeout: float = 10.0

    depth = 2.0 #depth to descend to
    heave_threshold = 0.1

    def handle_if_not_timedout(self) -> Union[Reached, None]:
        PIO.set_target_pose_heave(self.depth)

        if (PIO.is_heave_within_threshold(self.heave_threshold)):
            return self.Reached()
        else:
            return None
    
    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()
    

    
class DropMarker(TimedState):
    class DroppedLeft(NamedTuple): 
        pass
    class DroppedRight(NamedTuple): 
        pass
    class TimedOut(NamedTuple): 
        pass
    timeout: float = 10.0

    open_angle = 130 #angle to open servo to 

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        if not isinstance(prev_outcome, CenterOrSpin):
            self.drop_left = True
            #raise TypeError(f"Expected a CenterOrSpin outcome, received {prev_outcome}")
        else:
            self.drop_left = prev_outcome.is_center 
        
    def handle_if_not_timed_out(self) -> Union[DroppedLeft, DroppedRight]:
        if self.drop_left:
            left_dropper_pub = rospy.Publisher('/left_servo/angle', Int32)
            left_dropper_pub.publish(self.open_angle)
            return self.DroppedLeft()
        else:
            right_dropper_pub = rospy.Publisher('/right_servo/angle', Int32)
            right_dropper_pub.publish(self.open_angle)
            return self.DroppedRight()
    
    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()

class Spin180(TimedState):
    class Reached(CenterOrSpin): 
        pass
    class TimedOut(NamedTuple): 
        pass
    timeout: float = 10.0

    yaw_threshold = 2.5

    def __init__(self, prev_outcome):
        super().__init__(prev_outcome)
        self.target_yaw: float  = (PIO.Pose.yaw + 180) % 360

    def handle_if_not_timedout(self) -> Union[Reached, None]:
        PIO.set_target_pose_yaw(self.target_yaw)

        if (PIO.is_yaw_within_threshold(self.yaw_threshold)):
            return self.Reached(False)
        else:
            return None
    
    def handle_once_timedout(self) -> TimedOut:
        return self.TimedOut()
