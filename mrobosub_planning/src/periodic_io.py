import rospy
from std_msgs.msg import Float64, Bool
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse, PathmarkerAngle
from typing import Type, Mapping, Optional, Tuple
from enum import Enum

# TODO: where to put angle error and util repository?

# Don't look at these
# def _gate_position_callback(msg: Float64) -> None:
#     PIO.gate_position = msg
#
# def _gman_position_callback(msg : Float64) -> None:
#     PIO.gate_position = msg
#
# def _bootlegger_position_callback(msg : Float64) -> None:
#     PIO.gate_position = msg
#
# def _gun_position_callback(msg : Float64) -> None:
#     PIO.gun_position = msg
#
def _yaw_callback(msg : Float64) -> None:
    PIO.Pose.yaw = msg.data

def _heave_callback(msg : Float64) -> None:
    PIO.Pose.heave = msg.data

def _roll_callback(msg : Float64) -> None:
    PIO.Pose.roll = msg.data

def _collision_callback(msg : Bool) -> None:
    PIO.buoy_collision = msg.data

def angle_error(setpoint, state):
        return (setpoint - state + 180) % 360 - 180

Namespace = Type

Glyph = Enum('Glyph', [
    'abydos', 'earth',
    'taurus', 'serpens_caput', # 'capricornus', 'monoceros', 'sagittarius', 'orion', # abydos
    'auriga', 'cetus', # 'centaurus', 'cancer', 'scutum', 'eridanus', # earth
])

class Gbl:
    planet_seen: Optional[Glyph] = None
    first_hit_glyph: Optional[Glyph] = None
    second_glyph: bool = False

    @classmethod
    def glyphs_of_planet(cls, planet: Optional[Glyph]) -> Tuple[Glyph, Glyph]:
        if planet == Glyph.earth:
            return (Glyph.auriga, Glyph.serpens_caput)
        else:
            return (Glyph.taurus, Glyph.cetus)

    @classmethod
    def preferred_glyph(cls) -> Glyph:
        fst, snd = cls.glyphs_of_planet(Gbl.planet_seen)
        if Gbl.first_hit_glyph == fst:
            return snd
        elif Gbl.first_hit_glyph == snd:
            return fst
        else:
            return fst

# Look at this!
class PIO:
#public:
    # gate_position: float
    # gman_position: float
    # bootlegger_position: float
    # gun_position: float

    # # Output
    # heading_mode = HeadingRequest.DISABLED
    # heading_value = 0
    # target_depth = 0
    # forward = 0
    # lateral = 0
    # roll = 0

    class Pose:
        yaw = 0.0
        heave = 0.0
        roll = 0.0

    class TargetPose:
        yaw = 0.0
        heave = 0.0
        roll = 0.0
    
    buoy_collision = False

    # @classmethod
    # def heading_within_threshold(cls, threshold):
    #     return angle_error_abs(PIO.heading_value, PIO.current_heading) <= threshold

    @classmethod
    def is_yaw_within_threshold(cls, threshold):
        return abs(angle_error(cls.TargetPose.yaw, cls.Pose.yaw)) <= threshold

    @classmethod
    def is_heave_within_threshold(cls, threshold):
        return abs(cls.TargetPose.heave - cls.Pose.heave) <= threshold

    # @classmethod
    # def set_absolute_heading(cls, heading):
    #     PIO.heading_mode = HeadingRequest.ABSOLUTE
    #     PIO.heading_value = heading

    @classmethod
    def set_target_pose_yaw(cls, target_yaw : float) -> None:
       cls._target_pose_yaw_pub.publish(target_yaw)
       cls.TargetPose.yaw = target_yaw

    @classmethod
    def set_target_pose_heave(cls, target_heave: float) -> None:
        cls._target_pose_heave_pub.publish(target_heave)
        cls.TargetPose.heave = target_heave

    @classmethod
    def set_target_pose_roll(cls, target_roll: float) -> None:
        cls._target_pose_roll_pub.publish(target_roll)
        cls.TargetPose.roll = target_roll

    @classmethod
    def set_target_twist_roll(cls, override_roll : float) -> None:
        cls._target_twist_roll_pub.publish(override_roll)

    @classmethod
    def set_target_twist_yaw(cls, override_yaw : float) -> None:
        cls._target_twist_yaw_pub.publish(override_yaw)
    
    @classmethod
    def set_target_twist_surge(cls, override_surge : float) -> None:
        cls._target_twist_surge_pub.publish(override_surge)
    
    @classmethod
    def set_target_twist_sway(cls, override_sway : float) -> None:
        cls._target_twist_sway_pub.publish(override_sway)
    
    @classmethod
    def set_target_twist_heave(cls, override_heave: float) -> None:
        cls._target_twist_heave_pub.publish(override_heave)

    # @classmethod
    # def get_pose(cls) -> Namespace[Pose]:
    #     return cls.Pose
        
    @classmethod
    def query_pathmarker(cls) -> Optional[float]:
        """ Request a the pathmaker angle.

        Returns: 
            angle of path marker in global frame (i.e. same frame as the Pose.yaw) if found
            None otherwise. 
        """
        
        try:
            resp = cls._pathmarker_srv()
        except:
            return None

        if resp.found:
            return (90 - resp.angle) + cls.Pose.yaw
        else:
            return None

    @classmethod
    def query_glyph(cls, glyph: Optional[Glyph]) -> ObjectPositionResponse:
        try:
            return cls._object_position_srvs[glyph.name]()
        except:
            obj_msg = ObjectPositionResponse()
            obj_msg.found = False
            return obj_msg
 
    @classmethod
    def query_all_glyphs(cls) -> Mapping[Glyph, ObjectPositionResponse]:
        """ query all 12 glyphs and return a dict from any found glyphs to their position. """
        results = { }
        for g in Glyph:
            resp = cls.query_glyph(g)
            if resp.found:
                results[g] = resp
        return results

# private:

    # Subscribers
    rospy.Subscriber('/pose/yaw', Float64, _yaw_callback)
    rospy.Subscriber('/pose/heave', Float64, _heave_callback)
    rospy.Subscriber('/pose/roll', Float64, _roll_callback)
    rospy.Subscriber('/collision/collision', Bool, _collision_callback)


    # Publishers
    _target_pose_heave_pub = rospy.Publisher('/target_pose/heave', Float64, queue_size=1)
    _target_pose_yaw_pub = rospy.Publisher('/target_pose/yaw', Float64, queue_size=1)
    _target_pose_roll_pub = rospy.Publisher('/target_pose/roll', Float64, queue_size=1)

    _target_twist_yaw_pub = rospy.Publisher('/target_twist/yaw', Float64, queue_size=1)
    _target_twist_roll_pub = rospy.Publisher('/target_twist/roll', Float64, queue_size=1)
    _target_twist_surge_pub = rospy.Publisher('/target_twist/surge', Float64, queue_size=1)
    _target_twist_sway_pub = rospy.Publisher('/target_twist/sway', Float64, queue_size=1)
    _target_twist_heave_pub = rospy.Publisher('/target_twist/heave', Float64, queue_size = 1)
    
    # Services
    _pathmarker_srv = rospy.ServiceProxy('pathmarker/angle', PathmarkerAngle, persistent=True)
    _object_position_srvs = {}
    for g in Glyph:
        _object_position_srvs[g.name] = rospy.ServiceProxy(f'/object_position/{g.name}', ObjectPosition, persistent=True)
        
