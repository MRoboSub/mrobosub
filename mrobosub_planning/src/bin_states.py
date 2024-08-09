from umrsm import State
from abstract_states import TimedState
from periodic_io import PIO
from mrobosub_msgs.srv import ObjectPositionResponse
from typing import NamedTuple, Union
import math 
import rospy
from std_msgs.msg import Int32


class ApproachBinOpen(TimedState):
    class SeenBin(NamedTuple):
        pass

    class TimedOut(NamedTuple):
        pass

    surge_speed: float = 0.15
    timeout: float = 30

    def __init__(self, prev_outcome) -> None:
        super().__init__(prev_outcome)
        PIO.activate_bot_cam()
        self.target_yaw = getattr(prev_outcome, "angle", PIO.Pose.yaw)

    def handle_if_not_timedout(self) -> Union[SeenBin, None]:
        PIO.set_target_pose_yaw(self.target_yaw)
        PIO.set_target_twist_surge(self.surge_speed)

        binPosition = PIO.query_BinCamPos()
        if binPosition and binPosition.found:
            return self.SeenBin()
        return None

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        return self.TimedOut()

class ApproachBinClosed(TimedState):
    class TimedOut(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass

    timeout: float = 40.0

    surge_speed = .1 #max surge speed
    yaw_factor = .2
    centered_pixel_dist_thresh = .1 #threshold distance in pixels within which we say we have centered appropriatclass CenterToBinFromFar(TimedState):
    aligned_yaw_thresh = 5 #threshold within which we consider yaw aligned
    corrective_yaw_thresh = 30 #once angle to bin > thresh center the yaw
    yaw_aligned = False     #keeps track of if the yaw is aligned and we are suring or if we are not aligned and are aligning
    
    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        PIO.activate_bot_cam()
        self.angle_to_bin: float = 0.0


    def handle_if_not_timedout(self) -> Union[None, Reached]:
        bin_camera_position = PIO.query_BinCamPos() 
        #print(f"{bin_camera_position=}")
        dist_to_bin = 20000.
        if bin_camera_position and bin_camera_position.found:
            x, y = bin_camera_position.x_position, bin_camera_position.y_position
            x -= 0.5
            y -= 0.5
            y *= -1
            self.angle_to_bin = math.atan2(y, x) 
            self.angle_to_bin = (((math.degrees(self.angle_to_bin) - 90) % 360 + 180) % 360) - 180
            dist_to_bin = math.sqrt(y**2 + x**2)
            print(f'{x=:.2f}; {y=:.2f}; {self.angle_to_bin=:.2f}; {dist_to_bin=:.2f}; {self.yaw_aligned=}')

            if abs(self.angle_to_bin) > self.corrective_yaw_thresh:
                self.yaw_aligned = False
            elif abs(self.angle_to_bin) < self.aligned_yaw_thresh:
                self.yaw_aligned = True

            if not self.yaw_aligned:
                # Use setpoint for yaw angle
                PIO.set_target_twist_surge(0)
                PIO.set_target_pose_yaw((PIO.Pose.yaw % 360 - self.angle_to_bin * self.yaw_factor) % 360) ## adjust yaw factor in pool testing
            else:
                #PIO.set_target_pose_yaw(PIO.Pose.yaw)
                juice = (self.surge_speed*dist_to_bin) #set surge speed decreases as closer to centered
                print(f'{juice=}')
                PIO.set_target_twist_surge(juice)
            
        if dist_to_bin < self.centered_pixel_dist_thresh:
            PIO.set_target_twist_surge(0)
            PIO.set_target_pose_yaw(PIO.Pose.yaw)
            return self.Reached()
        else:
            return None
        

    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        PIO.set_target_pose_yaw(PIO.Pose.yaw)
        return self.TimedOut()

        

class CenterOrSpin(NamedTuple): #Used to mark which previous state we came from to allow for dropper code to work
    is_center: bool

class CenterCameraToBin(TimedState):
    class TimedOut(NamedTuple):
        pass
    class Reached(NamedTuple):
        pass
    timeout: float = 120.0

    surge_and_strafe_speed = 0.05 #max surge/sway speed
    centered_pixel_x_y_thresh = 0.15
    bin_depth = 0.5
    descend_speed = 0.05

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        PIO.activate_bot_cam()
        self.angle_to_bin: float = 0.0
        self.surge_speed = 0.0
        self.sway_speed = 0.0
        self.lost_frames_num = 0
        self.last_heave_target = PIO.Pose.heave
        self.i = 0

    def handle_if_not_timedout(self) -> Union[None, Reached]:
        if self.i < 30:
            self.i += 1
            PIO.set_target_twist_surge(-0.2)
        bin_camera_position = PIO.query_BinCamPos() 
        if bin_camera_position and bin_camera_position.found: 
            self.lost_frames_num = 0
            x,y = bin_camera_position.x_position, bin_camera_position.y_position
            x -= 0.5
            y -= 0.5
            y *= -1
            self.angle_to_bin = math.atan2(y, x) 

            surge_aligned = abs(y) < self.centered_pixel_x_y_thresh
            sway_aligned = abs(x) < self.centered_pixel_x_y_thresh

            if not surge_aligned:
                if y > 0:
                    self.surge_speed = self.surge_and_strafe_speed
                else:
                    self.surge_speed = -1 * self.surge_and_strafe_speed
                PIO.set_target_twist_surge(self.surge_speed)

            if not sway_aligned:
                if x > 0:
                    self.sway_speed = self.surge_and_strafe_speed
                else:
                    self.sway_speed = -1 * self.surge_and_strafe_speed
                PIO.set_target_twist_sway(self.sway_speed)

            print(f'{surge_aligned=}; {sway_aligned=}')
            if sway_aligned and surge_aligned:
                self.last_heave_target = max(self.last_heave_target, PIO.Pose.heave)
                PIO.set_target_twist_heave(self.descend_speed)
                print('Descending')
                if PIO.Pose.heave > self.bin_depth:
                    return self.Reached()
            else:
                PIO.set_target_pose_heave(self.last_heave_target)
        else:
            PIO.set_target_pose_heave(self.last_heave_target)
            self.lost_frames_num += 1
            if self.lost_frames_num > 10:
                PIO.set_target_twist_surge(-1*self.surge_speed)
                PIO.set_target_twist_sway(-1*self.sway_speed)

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
    dropper_offset = 0.05
    centered_pixel_x_y_thresh = 0.05 #threshold distance in pixels within which we say we have centered appropriatclass CenterToBinFromFar(TimedState):

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        PIO.activate_bot_cam()
        self.angle_to_bin: float = 0.0

    def handle_if_not_timedout(self) -> Union[None, Reached]:
        bin_camera_position = PIO.query_BinCamPos() 
        if bin_camera_position and bin_camera_position.found: 
            self.angle_to_bin = math.atan2(bin_camera_position.y, bin_camera_position.x) 

            surge_speed: float = 0.0
            sway_speed: float = 0.0
            
            if bin_camera_position.y - self.centered_pixel_x_y_thresh / 2 > 0:
                surge_speed = self.surge_and_strafe_speed
            elif bin_camera_position.y + self.centered_pixel_x_y_thresh / 2 < 0:
                surge_speed = -1 * self.surge_and_strafe_speed
            if bin_camera_position.x - self.centered_pixel_x_y_thresh / 2 - self.dropper_offset / 2 > 0:
                sway_speed = self.surge_and_strafe_speed
            elif bin_camera_position.x + self.centered_pixel_x_y_thresh / 2 - self.dropper_offset / 2 < 0:
                sway_speed = -1 * self.surge_and_strafe_speed

            PIO.set_target_twist_surge(surge_speed)
            PIO.set_target_twist_sway(sway_speed)

            if surge_speed == 0 and sway_speed == 0:
                return self.Reached(True)
            else:
                PIO.set_target_twist_heave(0)
        return None
            
    def handle_once_timedout(self) -> TimedOut:
        PIO.set_target_twist_surge(0)
        PIO.set_target_twist_sway(0)
        return self.TimedOut()

        
            
class Descend(TimedState):
    class Reached(NamedTuple): pass
    class TimedOut(NamedTuple): pass
    timeout: float = 10.0

    depth = 1.8 #depth to descend to
    heave_threshold = 0.2

    def handle_if_not_timedout(self) -> Union[Reached, None]:
        #PIO.set_target_pose_heave(self.depth)
        PIO.set_target_twist_heave(0.1)

        if PIO.Pose.heave > self.depth:
        #if (PIO.is_heave_within_threshold(self.heave_threshold)):
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

    def __init__(self, prev_outcome: NamedTuple):
        super().__init__(prev_outcome)
        if not isinstance(prev_outcome, CenterOrSpin):
            self.drop_left = True
            #raise TypeError(f"Expected a CenterOrSpin outcome, received {prev_outcome}")
        else:
            self.drop_left = prev_outcome.is_center 
        
    def handle_if_not_timedout(self) -> Union[DroppedLeft, DroppedRight]:
        if self.drop_left:
            PIO.set_left_dropper_angle(60)
            return self.DroppedLeft()
        else:
            PIO.set_right_dropper_angle(120)
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

    def __init__(self, prev_outcome: NamedTuple):
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
