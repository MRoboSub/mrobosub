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

<<<<<<< HEAD
    timeout: float = 40.0

    surge_speed = .6 #max surge speed
    yaw_factor = .2
    centered_pixel_dist_thresh = .1 #threshold distance in pixels within which we say we have centered appropriatclass CenterToBinFromFar(TimedState):
=======
    surge_speed = 0.2 #max surge speed
    yaw_factor = 1
    max_pixel_dist = 500.0 #maximum pixel distance we could see sqrt(maxPixelX**2+maxPixelY**2)
    centered_pixel_dist_thresh = 50 #threshold distance in pixels within which we say we have centered appropriatclass CenterToBinFromFar(TimedState):
>>>>>>> 4e8c80d2de434ba9abd1ef6cc9b8824948064250
    aligned_yaw_thresh = 5 #threshold within which we consider yaw aligned
    corrective_yaw_thresh = 30 #once angle to bin > thresh center the yaw
    yaw_aligned = False     #keeps track of if the yaw is aligned and we are suring or if we are not aligned and are aligning
    
    def __init__(self, prev_outcome):
        super().__init__(prev_outcome)
        self.angle_to_bin: float = 0.0


    def handle_if_not_timedout(self) -> Union[None, Reached]:
        bin_camera_position = PIO.query_BinCamPos() 
        #print(f"{bin_camera_position=}")
        dist_to_bin = 20000
        if bin_camera_position and bin_camera_position.found:
            x, y = bin_camera_position.x_position, bin_camera_position.y_position
            x -= 0.5
            y -= 0.5
            y *= -1
            self.angle_to_bin = math.atan2(y, x) 
<<<<<<< HEAD
            self.angle_to_bin = (((math.degrees(self.angle_to_bin) - 90) % 360 + 180) % 360) - 180
            dist_to_bin = math.sqrt(y**2 + x**2)
            print(f'{x=:.2f}; {y=:.2f}; {self.angle_to_bin=:.2f}; {dist_to_bin=:.2f}; {self.yaw_aligned=}')
=======
            self.angle_to_bin = math.degrees(-self.angle_to_bin + math.pi/2)
            self.dist_to_bin = math.sqrt(y**2 + x**2)
>>>>>>> 4e8c80d2de434ba9abd1ef6cc9b8824948064250

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
    timeout: float = 40.0

<<<<<<< HEAD
    surge_and_strafe_speed = 0.05 #max surge/sway speed
    centered_pixel_x_y_thresh = 0.05 
=======
    surge_and_strafe_speed = 0.2 #max surge/sway speed
    yaw_factor = 1
    max_pixel_dist = 500.0 #maximum pixel distance we could see sqrt(maxPixelX**2+maxPixelY**2)
    centered_pixel_x_y_thresh = 50 #threshold distance in pixels if x is within then no sway and if y is within then no y
    surge_aligned = False
    sway_aligned = False
>>>>>>> 4e8c80d2de434ba9abd1ef6cc9b8824948064250

    def __init__(self, prev_outcome):
        super().__init__(prev_outcome)
        self.angle_to_bin: float = 0.0

    def handle_if_not_timedout(self) -> Union[None, Reached]:
        bin_camera_position = PIO.query_BinCamPos() 
        if bin_camera_position and bin_camera_position.found: 
            x,y = bin_camera_position.x_position, bin_camera_position.y_position
<<<<<<< HEAD
            x -= 0.5
            y -= 0.5
            y *= -1
            self.angle_to_bin = math.atan2(y, x) 

            surge_aligned = abs(y) < self.centered_pixel_x_y_thresh
            sway_aligned = abs(x) < self.centered_pixel_x_y_thresh

            if not surge_aligned:
                if y > 0:
                    surge_speed = self.surge_and_strafe_speed
                else:
                    surge_speed = -self.surge_and_strafe_speed
                PIO.set_target_twist_surge(surge_speed)

            if not sway_aligned:
                if x > 0:
                    sway_speed = self.surge_and_strafe_speed
                else:
                    sway_speed = -self.surge_and_strafe_speed
                PIO.set_target_twist_sway(sway_speed)

            print(f'{surge_aligned=}; {sway_aligned=}')
            if sway_aligned and surge_aligned:
                #return self.Reached()
                PIO.set_target_twist_heave(0.1)
                print('Descending')
                if PIO.Pose.heave > 1.8:
                    return self.Reached()
=======
            self.angle_to_bin = math.atan2(y, x) 

            surge_speed:float = 0.0
            sway_speed:float = 0.0

            if not self.surge_aligned:
                if y - self.centered_pixel_x_y_thresh / 2 > 0:
                    surge_speed = self.surge_and_strafe_speed
                elif y + self.centered_pixel_x_y_thresh / 2 < 0:
                    surge_speed = -1 * self.surge_and_strafe_speed
                else:
                    self.surge_aligned = True
                    if abs(x) > self.centered_pixel_x_y_thresh/2:
                        self.sway_aligned = False

                PIO.set_target_twist_surge(surge_speed)

            elif not self.sway_aligned:
                if x - self.centered_pixel_x_y_thresh / 2 > 0:
                    sway_speed = self.surge_and_strafe_speed
                elif x + self.centered_pixel_x_y_thresh / 2 < 0:
                    sway_speed = -1 * self.surge_and_strafe_speed
                else:
                  self.sway_alinged = True
                  if abs(y) > self.centered_pixel_x_y_thresh/2:
                    self.surge_aligned = False

                PIO.set_target_twist_sway(sway_speed)

            if self.sway_aligned and self.surge_aligned:
                return self.Reached()
>>>>>>> 4e8c80d2de434ba9abd1ef6cc9b8824948064250
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
