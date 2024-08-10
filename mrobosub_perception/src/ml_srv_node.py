#!/usr/bin/env python

from functools import partial
from typing import List, Optional

import rospy
import rospkg
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse
import cv2
from cv_bridge import CvBridge
# from zed_interfaces.msg import RGBDSensors
import numpy as np
import time
import sys
import enum
import pathlib
import torch
#from rsub_log import log
#from get_depth import get_avg_depth
import struct
import os
from sensor_msgs.msg import Image

# height = 376
# width = 1344
# const_unpack = ''
# for i in range(376*1344*2):
    # const_unpack += 'B'

bridge = CvBridge()

#log("ml_node", "INFO", "Starting")
#log("ml_node", "DEBUG", 'python version: ' + sys.version)
#log("ml_node", "DEBUG", 'cv2 version: ' + cv2.__version__)
#log("ml_node", "DEBUG", 'cv2 location:'+ cv2.__file__)

CONFIDENCE = 0.9
TIME_THRESHOLD = 10

class Targets(enum.Enum):
    GATE_RED = 0
    GATE_BLUE = 1

def load_yolo():
    # load model
    if torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")

    path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    yolo_path = os.path.join(path, 'yolov5')

    model_path = os.path.join(path, "models/comp_gate_2024.pt")
    model = torch.hub.load(yolo_path, 'custom', path=model_path, source='local')  # local repo
    model.conf = 0.1  # NMS confidence threshold
    return model

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv[:,:,:3]

class Node:
    recent_positions: List[Optional[ObjectPositionResponse]]
    red_is_left: bool = True

    def __init__(self):
        # Load the model
        #model, classes, colors, output_layers = load_yolo()
        self.model = load_yolo()
        print("model loaded")

        print("made it to main")
        rospy.init_node('ml_server', anonymous=False)
        print(sys.version)
        print("node initialized")
        
        # Intialize ros services for each of the objects
        mk_service = lambda name, idx: rospy.Service(
            f'object_position/{name}', 
            ObjectPosition, 
            lambda msg : self.handle_obj_request(idx.value, msg)
        )
        self.gate_red_srv = mk_service('gate_red', Targets.GATE_RED)
        self.gate_blue_srv = mk_service('gate_blue', Targets.GATE_BLUE)

        self.recent_positions = [None] * len(Targets)
        self.latest_request_time = rospy.get_time() - TIME_THRESHOLD

        # Subscribe to the ZED left image and depth topics
        # For a full list of zed topics, see https://www.stereolabs.com/docs/ros/zed-node/#published-topics
        rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, self.zed_callback, queue_size=1)

        self.bbox_pub = rospy.Publisher('/object_position/bbox', Image, queue_size=10)


    def zed_callback(self, message):
        global img_seen_num, img_num
        recent_positions_local: List[Optional[ObjectPositionResponse]] = [None] * len(Targets)

        if self.latest_request_time is None:
            return
        if rospy.get_time() - self.latest_request_time < TIME_THRESHOLD:
        # if True:
            start = time.time()

            # print("zed callback")
            # Convert the zed image to an opencv image
            # image_ocv = imgmsg_to_cv2(message)
            image_ocv = bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')

            # Remove the 4th channel (transparency)
            # image_ocv = image_ocv[:,:,:3]

            # Find any objects in the image
            height, width, channels = image_ocv.shape   # shape of the image
            outputs = self.model(image_ocv, size=width)   # get raw detection data

            detections = outputs.xyxy[0].cpu().numpy()   # get the detections

            # Draw any bounding boxes and display the image
            # results.show()

            #byte_data = struct.unpack(const_unpack, message.depth.data)

            # Report detection result
            print(detections)
            print('TIME: ', str((time.time() - start)))

            for i in range(len(recent_positions_local)):
                msg = ObjectPositionResponse()
                msg.found = False
                recent_positions_local[i] = msg
            
            bboxs = [None] * len(recent_positions_local)

            for i, detection in enumerate(detections):
                object_position_response = ObjectPositionResponse()
                object_position_response.found = True
            
                box = detection[:4]
                fov_x = 110
                fov_y = 70

                bbox_width = abs(box[0] - box[2])
                bbox_height = abs(box[1] - box[3])
                bbox_area = (bbox_width * bbox_height) / (width * height)

                x_pos = int((box[0] + box[2]) / 2)
                y_pos = int((box[1] + box[3]) / 2)
                
                d_x = x_pos - (width / 2)
                d_y = y_pos - (height / 2)
                
                theta_x = (d_x * fov_x) / width
                theta_y = (d_y * fov_y) / height
                
                conf = detection[4]
                
                print(x_pos, y_pos, theta_x, theta_y)
            
                object_position_response.found      = True
                # object_position_response.x_position = x_pos
                object_position_response.x_position = bbox_area # TODO: change this. for competition
                object_position_response.y_position = y_pos
                object_position_response.x_theta    = theta_x
                object_position_response.y_theta    = theta_y
                object_position_response.confidence = conf
            
            
                idx = int(detection[5])

                if idx < 2: # for red and blue gate symbols
                    if recent_positions_local[0] and recent_positions_local[0].found:
                        if(recent_positions_local[0].x_theta < object_position_response.x_theta):
                            if(self.red_is_left):
                                idx = 1
                            else:
                                idx = 0
                        else:
                            if(self.red_is_left):
                                idx = 0
                            else:
                                idx = 1
                    elif recent_positions_local[1] and recent_positions_local[1].found:
                        if(recent_positions_local[1].x_theta < object_position_response.x_theta):
                            if(self.red_is_left):
                                idx = 1
                            else:
                                idx = 0
                        else:
                            if(self.red_is_left):
                                idx = 0
                            else:
                                idx = 1
                    
                if recent_positions_local[idx] and recent_positions_local[idx].found:
                    recent_positions_local[idx], recent_positions_local[1-idx] = object_position_response, recent_positions_local[idx]
                    bboxs[idx], bboxs[1-idx] = box, bboxs[idx]
                else:
                    recent_positions_local[idx] = object_position_response
                    bboxs[idx] = box

                # draw bounding box on image
            
            for idx, box in enumerate(bboxs):
                if box is None:
                    continue
                cv2.rectangle(image_ocv, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 255, 255), 2)
                cv2.putText(image_ocv, f"{Targets(idx).name} {conf:.2f}", (int(box[0]), int(box[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255))

            self.recent_positions = recent_positions_local
            
            print(f"processed in {time.time() - start}")

            msg = bridge.cv2_to_imgmsg(image_ocv, encoding='rgb8')
            self.bbox_pub.publish(msg)


            # THIS PRINTS a lOT
            # log("ml_node", "DEBUG", 'ml_node - done processing a frame in ' + str((time.time()-start)) + ' seconds')
        else:
            for i in range(len(Targets)):
                self.recent_positions[i] = None


    def handle_obj_request(self, idx, msg):
        self.latest_request_time = rospy.get_time()
        while self.recent_positions[idx] == None:
            rospy.sleep(0.005)
        return self.recent_positions[idx]


    def spin(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    Node().spin()
