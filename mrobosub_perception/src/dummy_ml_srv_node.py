#!/usr/bin/env python

from functools import partial

import rospy
import rospkg
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse
import cv2
import numpy as np
import time
import sys
import enum
import pathlib
import struct
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


bridge = CvBridge()


CONFIDENCE = 0.9
TIME_THRESHOLD = 10


class PeriodicIO():
    bbox_pub = None


class Targets(enum.Enum):
    ABYDOS = 0
    EARTH = 1
    TAURUS = 2
    SERPENS_CAPUT = 3
    AURIGA = 4
    CETUS = 5

recent_positions = [None] * len(Targets)
latest_request_time = None
def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv[:,:,:3]

counters = [0] * len(Targets)
def handle_obj_request(idx, msg):
    global counters
    obj_msg = ObjectPositionResponse()
    obj_msg.found = counters[idx] > 10 
    counters[idx] += 1
    return obj_msg

if __name__ == '__main__':
    print("made it to main")
    rospy.init_node('ml_server', anonymous=False)
    print(sys.version)
    print("node initialized")

    # Intialize ros services for each of the objects
    mk_service = lambda name, idx: rospy.Service(f'object_position/{name}', ObjectPosition, lambda msg : handle_obj_request(idx.value, msg))
    abydos_srv = mk_service('abydos',Targets.ABYDOS)
    earth_srv = mk_service('earth',Targets.EARTH)
    taurus_srv = mk_service('taurus',Targets.TAURUS)
    serpens_caput_srv = mk_service('serpens_caput',Targets.SERPENS_CAPUT)
    auriga_srv = mk_service('auriga',Targets.AURIGA)
    cetus_srv = mk_service('cetus',Targets.CETUS)

    rospy.spin()
