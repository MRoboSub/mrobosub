#!/usr/bin/env python

from enum import Enum, auto
import gc
from dataclasses import dataclass
from typing import Dict, List
from std_msgs.msg import Float64, String
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse, BinCamPos, BinCamPosResponse  # type: ignore
from time import sleep
from functools import partial
import rospy


class Glyph(Enum):
    ABYDOS = auto()
    EARTH = auto()
    TAURUS = auto()
    SERPENS_CAPUT = auto()
    AURIGA = auto()
    CETUS = auto()


class binCamPosServiceMock:
    def __init__(self) -> None:
        self.position = BinCamPosResponse() ## dummy position
        self.service = rospy.Service('bin_cam_pos', BinCamPos, self.handle_obj_request)

    def handle_obj_request(self, _msg):
        return self.position
    
    def set_position(self, pos: BinCamPosResponse):
        self.position = pos

class ObjectPositionServiceMock:
    def __init__(self) -> None:
        self._visible_glyphs: Dict[Glyph, ObjectPositionResponse] = {}
        mk_service = lambda name, glyph: rospy.Service(f'object_position/{name}', ObjectPosition, lambda _msg : self._handle_obj_request(glyph))
        self.services = {glyph: mk_service(glyph.name.lower(), glyph) for glyph in Glyph}

    def _handle_obj_request(self, target: Glyph):
        res = ObjectPositionResponse()
        res.found = False
        return self._visible_glyphs.get(target, res)

    def set_seen_glyphs(self, positions: Dict[Glyph, ObjectPositionResponse]):
        self._visible_glyphs = positions

class PoseMock:
    def __init__(self) -> None:
        self.heave = 0.
        self.roll = 0.
        self.yaw = 0.

        self._heave_pub = rospy.Publisher('/pose/heave', Float64, queue_size=1)
        self._roll_pub = rospy.Publisher('/pose/roll', Float64, queue_size=1)
        self._yaw_pub = rospy.Publisher('/pose/yaw', Float64, queue_size=1)

    def publish_update(self) -> None:
        self._heave_pub.publish(self.heave)
        self._roll_pub.publish(self.roll)
        self._yaw_pub.publish(self.yaw)

@dataclass
class TargetReader:
    target_pose_heave: int = 0.
    target_pose_roll: int = 0.
    target_pose_yaw: int = 0.
    target_pose_surge: int = 0.
    target_pose_sway: int = 0.
    target_pose_pitch: int = 0.

    target_twist_heave: int = 0.
    target_twist_roll: int = 0.
    target_twist_yaw: int = 0.
    target_twist_surge: int = 0.
    target_twist_sway: int = 0.
    target_twist_pitch: int = 0.

    def __init__(self, *args, **kwargs) -> None:

        self._pose_heave_sub = rospy.Subscriber('/target_pose/heave', Float64, partial(self._pose_callback, 'heave'))
        self._pose_roll_sub = rospy.Subscriber('/target_pose/roll', Float64, partial(self._pose_callback, 'roll'))
        self._pose_yaw_sub = rospy.Subscriber('/target_pose/yaw', Float64, partial(self._pose_callback, 'yaw'))

        self._twist_heave_sub = rospy.Subscriber('/target_twist/heave', Float64, partial(self._twist_callback, 'heave'))
        self._twist_roll_sub = rospy.Subscriber('/target_twist/roll', Float64, partial(self._twist_callback, 'roll'))
        self._twist_yaw_sub = rospy.Subscriber('/target_twist/yaw', Float64, partial(self._twist_callback, 'yaw'))
        self._twist_surge_sub = rospy.Subscriber('/target_twist/surge', Float64, partial(self._twist_callback, 'surge'))
        self._twist_sway_sub = rospy.Subscriber('/target_twist/sway', Float64, partial(self._twist_callback, 'sway'))
        self._twist_pitch_sub = rospy.Subscriber('/target_twist/pitch', Float64, partial(self._twist_callback, 'pitch'))

    def _pose_callback(self, key: str, msg: Float64) -> None:
        setattr(self, f'target_pose_{key}', msg.data)
        # setattr(self, f'target_twist_{key}', None)

    def _twist_callback(self, key: str, msg: Float64) -> None:
        setattr(self, f'target_twist_{key}', msg.data)
        # setattr(self, f'target_pose_{key}', None)

def main():
    rospy.init_node('test_sim')
    started = False
    sub = None
    def set_started(msg: String):
        nonlocal started, sub
        if started:
            return
        print(msg.data)
        started = True
        if sub is not None:
            sub.unregister()
        del sub
    sub = rospy.Subscriber('/captain/current_state', String, set_started)

    while not started:
        sleep(0.1)
    print('Started!')
    sleep(0.5)

    ml_srvs = ObjectPositionServiceMock()
    pose = PoseMock()
    target = TargetReader()

    pose.heave = 0.
    while pose.heave < target.target_pose_heave:
        pose.heave += 0.005
        sleep(0.05)
        pose.publish_update()

    for i in range(20*3):
        sleep(0.05)

    seen = ObjectPositionResponse()
    seen.found = True
    seen.x_position = 0.000001
    seen.y_position = 23.
    seen.x_theta = -5.
    seen.y_theta = 0.
    seen.confidence = 1.
    ml_srvs.set_seen_glyphs({Glyph.EARTH: seen})

    for i in range(20*5):
        sleep(0.05)

    ml_srvs.set_seen_glyphs({})

    for i in range(20*8):
        sleep(0.05)

    ml_srvs.set_seen_glyphs({Glyph.ABYDOS: seen})

    for i in range(20*60):
        sleep(0.05)


if __name__ == '__main__':
    main()
