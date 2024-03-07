from enum import Enum, auto
import gc
from utils import binCamPosServiceMock, TargetReader
from typing import Dict, List
from std_msgs.msg import Float64, String
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse, BinCamPosResponse  # type: ignore
from time import sleep
from functools import partial
import rospy

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

    bin_cam_mock = binCamPosServiceMock()
    reader = TargetReader()

    bin_cam_instance = BinCamPosResponse()
    bin_cam_instance.found = True
    bin_cam_instance.x = -150
    bin_cam_instance.y = 150

    bin_cam_mock.set_position(bin_cam_instance)

    for i in range(20*20):
        sleep(0.05)
        print(f"{reader=}")
        print(f"{reader.target_twist_yaw=}")



if __name__ == '__main__':
    main()