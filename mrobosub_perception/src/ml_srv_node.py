#!/usr/bin/env python

from functools import partial

import rospy
import rospkg
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse
import cv2
from zed_interfaces.msg import RGBDSensors
from cv_bridge import CvBridge
import numpy as np
import time
import sys
import enum
import pathlib
import torch
#from rsub_log import log
#from get_depth import get_avg_depth
import struct
from sensor_msgs.msg import Image

height = 376
width = 1344
bridge = CvBridge()
const_unpack = ''
for i in range(height*width*2):
    const_unpack += 'B'


#log("ml_node", "INFO", "Starting")
#log("ml_node", "DEBUG", 'python version: ' + sys.version)
#log("ml_node", "DEBUG", 'cv2 version: ' + cv2.__version__)
#log("ml_node", "DEBUG", 'cv2 location:'+ cv2.__file__)

CONFIDENCE = 0.9
TIME_THRESHOLD = 10


class Targets(enum.Enum):
    GATE = 0
    GMAN = 1
    BOOTLEGGER = 2
    GUN = 3
    BADGE = 4
    COUNT = 5

recent_positions = [None] * Targets.COUNT.value
latest_request_time = None

def load_yolo():
    # load model
    if torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")
    model = torch.hub.load('./yolov5', 'custom', path='./models/best.pt', source='local')  # local repo
    model.conf = 0.25  # NMS confidence threshold
    return model

def zed_callback(message):
    global img_seen_num, img_num
    # print('in zed callback')
    if rospy.get_time() - latest_request_time < TIME_THRESHOLD:
    # if True:
        print('processing img')

        # print("zed callback")
        # Convert the zed image to an opencv image
        bridge = CvBridge()
        image_ocv = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')

        # Remove the 4th channel (transparency)
        image_ocv = image_ocv[:,:,:3]

        # Find any objects in the image
        height, width, channels = image_ocv.shape   # shape of the image
        outputs = model(image_ocv, size=width)   # get raw detection data
        detections = outputs.xyxy[0].numpy()   # get the detections

        # Draw any bounding boxes and display the image
        # results.show()

        object_position_response = ObjectPositionResponse()


        #byte_data = struct.unpack(const_unpack, message.depth.data)

        # Report detection result
        for i, id in enumerate(class_ids):
            object_position_response.found = True

            box = boxes[i] # grab the first box in the list
            if(len(box) == 0):
                object_position_response.found = False
            else:
                object_position_response.x_percent = float(box[0]) / (width + 1e-10) # TODO: this should not cause an error
                object_position_response.y_percent = float(box[1]) / (height + 1e-10)
                object_position_response.x_diff = (float(box[0]) - (width / 2)) / (width / 2)
                object_position_response.y_diff = ((height / 2) - float(box[1])) / (height / 2)
                object_position_response.box_size = float(box[2]) / width
                object_position_response.distance = 0.0  # TODO: fix get_avg_depth(message.depth, box)
                object_position_response.confidence = confs[i]
                # print(id)
                # print("confidence: " + str(object_position.confidence))
            recent_positions[id] = object_position_response
            print(id)
            print(recent_positions[id])

        # THIS PRINTS a lOT
        # log("ml_node", "DEBUG", 'ml_node - done processing a frame in ' + str((time.time()-start)) + ' seconds')
    else:
        for i in range(Targets.COUNT.value):
            recent_positions[i] = None


def handle_obj_request(idx, msg):
    breakpoint()
    global latest_request_time
    latest_request_time = rospy.get_time()
    while recent_positions[idx] == None:
        rospy.sleep(5)
    return recent_positions[idx]

# Load the model
#model, classes, colors, output_layers = load_yolo()
model, output_layers = load_yolo()
obj_pos_pub = None
bounding_pub = None
print("model loaded")

if __name__ == '__main__':
    print("made it to main")
    rospy.init_node('ml_server', anonymous=False)
    print("node initialized")

    # Intialize ros services for each of the objects
    mk_service = lambda name, idx: rospy.Service(f'object_position/{name}', ObjectPosition, lambda msg : handle_obj_request(idx.value, msg))
    gate_srv = mk_service('gate',Targets.GATE)
    gman_srv = mk_service('gman',Targets.GMAN)
    bootlegger_srv = mk_service('bootlegger',Targets.BOOTLEGGER)
    gun_srv = mk_service('gun',Targets.GUN)
    badge_srv = mk_service('badge',Targets.BADGE)

    for i in range(Targets.COUNT.value):
        recent_positions[i] = None
    latest_request_time = rospy.get_time() - TIME_THRESHOLD

    # Subscribe to the ZED left image and depth topics
    # For a full list of zed topics, see https://www.stereolabs.com/docs/ros/zed-node/#published-topics
    rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, zed_callback, queue_size=1)
    #rospy.Subscriber("/zed_nodelet/rgb/image_rect_color", Image, zed_callback, queue_size=1)
    rospy.spin()

    cv2.destroyAllWindows()

