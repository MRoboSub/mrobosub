import gc
from enum import Enum, auto
from functools import partial
from time import sleep
from typing import Dict, List

import rospy
from std_msgs.msg import Float64, String
from utils import PoseMock, TargetReader, binCamPosServiceMock

from mrobosub_msgs.srv import (  # type: ignore
    BinCamPosResponse,
    ObjectPosition,
    ObjectPositionResponse,
)


def main() -> None:
    rospy.init_node("test_sim")
    started = False
    sub = None

    def set_started(msg: String) -> None:
        nonlocal started, sub
        if started:
            return
        print(msg.data)
        started = True
        if sub is not None:
            sub.unregister()
        del sub

    sub = rospy.Subscriber("/captain/current_state", String, set_started)

    while not started:
        sleep(0.1)
    print("Started!")
    sleep(0.5)

    bin_cam_mock = binCamPosServiceMock()
    reader = TargetReader()
    pose = PoseMock()

    pose.heave = 0.0

    while pose.heave < reader.target_pose_heave:
        pose.heave += 0.005
        # print(f"{pose.heave=}")
        sleep(0.05)
        pose.publish_update()

    for i in range(20 * 3):
        sleep(0.05)

    bin_cam_instance = BinCamPosResponse()
    bin_cam_instance.found = True
    bin_cam_instance.x = -100
    bin_cam_instance.y = 173

    bin_cam_mock.set_position(bin_cam_instance)

    pose.yaw = 50
    pose.publish_update()

    for i in range(20 * 3):

        sleep(0.05)

        # print(f"{reader=}")
        print(f"{reader.target_pose_yaw=}")

    # bin_cam_instance.x *= -1
    # bin_cam_mock.set_position(bin_cam_instance)

    # for i in range(20*3):
    #     sleep(0.05)
    #     #print(f"{reader=}")
    #     print(f"{reader.target_pose_yaw=}")


if __name__ == "__main__":
    main()
