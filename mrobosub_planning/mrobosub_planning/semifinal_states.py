from mrobosub_planning.periodic_io import SegImageTarget, ImageTarget
from mrobosub_planning.abstract_states import AlignPathmarker, CenterOnPathmarker, ForwardAndWait, TimedState, TurnToYaw
import rclpy
from mrobosub_planning.umrsm import State, Outcome

from typing import Optional, List, Union
class TurnToGate(TurnToYaw):
    class Reached(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    target_yaw: float = 0.0
    yaw_threshold: float = 3.0
    settle_time: float = 5.0
    timeout: float = 10.0

    def handle_reached(self) -> Outcome:
        return self.Reached()
    
    def handle_unreached(self) -> Outcome:
        return self.TimedOut()
    

class SemiAlignPathmarker(AlignPathmarker):
    class Aligned(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    def handle_aligned(self) -> Outcome:
        self.io_node.deactivate_bot_cam()
        return self.Aligned()
    
    def handle_timedout(self) -> Outcome:
        self.io_node.deactivate_bot_cam()
        return self.TimedOut()
    
class SemiCenterOnPathmarker(CenterOnPathmarker):
    class Centered(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    def handle_centered(self) -> Outcome:
        return self.Centered()

    def handle_timedout(self) -> Outcome:
        return self.TimedOut()

class AlignGate(TimedState):
    class Aligned(Outcome):
        pass
    class TimedOut(Outcome):
        pass

    def handle_no_measurements(self) -> Outcome:
        self.io_node.deactivate_zed()
        return self.TimedOut()

    def handle_aligned(self) -> Outcome:
        self.io_node.deactivate_zed()
        return self.Aligned()

    timeout: float = 8
    yaw_threshold: float = 2.

    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node) -> None:
        super().__init__(prev_outcome, node)
        self.io_node.activate_zed()
        self.last_known_angle: Optional[float] = None
        self.iter = 0
        self.measurements: List[float] = []

    def handle_if_not_timedout(self) -> Union[Outcome, None]:
        self.io_node.set_target_twist_surge(0)
        self.iter += 1
        if self.iter < 50:
            return None
        if self.iter < 100:
            pm_resp = self.io_node.query_image(ImageTarget.GATE_SOS)
            self.io_node.get_logger().info({f"{pm_resp=}"})
            if pm_resp is not None and pm_resp.found and pm_resp.valid:
                self.measurements.append(pm_resp)
            return None
        if self.iter == 100:
            self.io_node.get_logger().info({"Calculating target"})
            if len(self.measurements) < 20:
                return self.handle_no_measurements()
            self.target_angle = sum(self.measurements) / len(self.measurements)
            self.io_node.get_logger().info({f"{self.target_angle=}"})
            self.yaw_threshold_count = 0
        if self.iter >= 100:
            self.io_node.set_target_pose_yaw(self.target_angle)
            if self.io_node.is_yaw_within_threshold(self.yaw_threshold):
                self.yaw_threshold_count += 1
            else:
                self.yaw_threshold_count = 0
            if self.yaw_threshold_count > 30:
                return self.handle_aligned()
        return None
    
    def handle_timedout(self) -> Outcome:
        self.io_node.deactivate_zed()
        return self.TimedOut()
    
class FindPathmarker(TimedState):
    class Found(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    def handle_no_measurements(self) -> Outcome:
        return self.TimedOut()

   
    def handle_found(self) -> Outcome:
        return self.Found()

    timeout: float = 20.0
    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node) -> None:
        super().__init__(prev_outcome, node)
        self.io_node.activate_bot_cam()
        self.num_spotted = 0

    def handle_if_not_timedout(self) -> Union[Outcome, None]:
        self.io_node.set_target_twist_surge(0.3)
       
        pm_resp = self.io_node.query_seg_image(SegImageTarget.PATHMARKER)
        self.io_node.get_logger().info({f"{pm_resp=}"})

        if pm_resp is not None and pm_resp.found and pm_resp.valid:
            self.num_spotted += 1

        if self.num_spotted > 30:
            return self.handle_found()
        return None
    
    def handle_timedout(self) -> Outcome:
        return self.TimedOut()
    
# class FindSlalom (TimedState):
#     class Found(Outcome):
#         pass

#     class TimedOut(Outcome):
#         pass

#     def handle_no_measurements(self) -> Outcome:
#         return self.TimedOut()

   
#     def handle_found(self) -> Outcome:
#         return self.Found()

#     timeout: float = 20.0
#     def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node) -> None:
#         super().__init__(prev_outcome, node)
#         self.io_node.activate_bot_cam()
#         self.num_spotted = 0

#     def handle_if_not_timedout(self) -> Union[Outcome, None]:
#         self.io_node.set_target_twist_surge(0.3)
       
#         pm_resp = self.io_node.query_image(ImageTarget.SLALOM)
#         self.io_node.get_logger().info({f"{pm_resp=}"})
#         if pm_resp is not None and pm_resp.found and pm_resp.valid:
#             self.num_spotted += 1

#         if self.num_spotted > 30:
#             return self.handle_found()
#         return None
    
#     def handle_timedout(self) -> Outcome:
#         return self.TimedOut()
    
class NavigateSlalom (TimedState):
    class Navigated(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    # def handle_no_measurements(self) -> Outcome:
    #     return self.TimedOut()

   
    # def handle_navigated(self) -> Outcome:
    #     return self.Navigated()

    timeout: float = 20.0
    scan_counter: int = 0
    scan_val = 0.2
    def __init__(self, prev_outcome: Outcome, node: rclpy.node.Node) -> None:
        super().__init__(prev_outcome, node)
        self.io_node.activate_zed()
        self.scan_counter = 0

    def handle_if_not_timedout(self) -> Union[Outcome, None]:
        
        rp_msg = self.io_node.query_image(ImageTarget.RED_POLE)
        wp_msg = self.io_node.query_image(ImageTarget.WHITE_POLE)
        rp_found = rp_msg is not None and rp_msg.found and rp_msg.valid
        wp_found = wp_msg is not None and wp_msg.found and wp_msg.valid

        if (rp_found and wp_found):
            self.scan_counter = 0
            rp_x = rp_msg.x_theta
            wp_x = wp_msg.x_theta
            target_x = (rp_x + wp_x) / 2.0
            self.io_node.set_target_twist_yaw(target_x * (-0.05)) 
            self.io_node.set_target_twist_surge(0.6)
            return None

        elif (rp_found):
            self.scan_counter = 0
            self.io_node.set_target_twist_yaw(-0.2) 
            self.io_node.set_target_twist_surge(0.1)
            return None
        else:
            self.scan_counter += 1
            if (self.scan_counter > 50):
                self.scan_counter = 0
                self.scan_val = -self.scan_val
            self.io_node.set_target_twist_yaw(self.scan_val) 
            self.io_node.set_target_twist_surge(0.01)
            return None

    def handle_timedout(self) -> Outcome:
        return self.TimedOut()
    
class GoToOctagon (TimedState):
    class Reached(Outcome):
        pass

    class TimedOut(Outcome):
        pass

    timeout: float = 20.0

    def handle_if_not_timedout(self) -> Union[Outcome, None]:
        pinger = self.io_node.query_image(ImageTarget.PINGER)
        octagon = self.io_node.query_image(ImageTarget.OCTAGON)
        pinger_found = pinger is not None and pinger.found and pinger.valid
        octagon_found = octagon is not None and octagon.found and octagon.valid

        if(pinger_found):
            self.io_node.set_target_twist_yaw(pinger.x_theta * (-0.05)) 
            self.io_node.set_target_twist_surge(0.6)
            return None
        
        elif(octagon_found):
            self.io_node.set_target_twist_yaw(octagon.x_theta * (-0.05)) 
            self.io_node.set_target_twist_surge(0.6)
            return None
        
        else:
            self.io_node.set_target_twist_yaw(-0.001) 
            self.io_node.set_target_twist_surge(0.5)
            return None

    
    def handle_timedout(self) -> Outcome:
        return self.TimedOut()