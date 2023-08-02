#!/usr/bin/env python

from functools import partial

import rospy
import rospkg
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse
import cv2
from zed_interfaces.msg import RGBDSensors
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
from cv_bridge import CvBridge

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

def load_yolo():
    # load model
    if torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")

    path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    yolo_path = os.path.join(path, 'yolov5')

    model_path = os.path.join(path, "models/model_2a.pt")
    model = torch.hub.load(yolo_path, 'custom', path=model_path, source='local')  # local repo
    model.conf = 0.25  # NMS confidence threshold
    return model

def zed_callback(message):
    global img_seen_num, img_num
    # print('in zed callback')
    if rospy.get_time() - latest_request_time < TIME_THRESHOLD:
    # if True:
        start = time.time()

        # print("zed callback")
        # Convert the zed image to an opencv image
        image_ocv = imgmsg_to_cv2(message)

        # Remove the 4th channel (transparency)
        # image_ocv = image_ocv[:,:,:3]

        # Find any objects in the image
        height, width, channels = image_ocv.shape   # shape of the image
        outputs = model(image_ocv, size=width)   # get raw detection data
        detections = outputs.xyxy[0].cpu().numpy()   # get the detections

        # Draw any bounding boxes and display the image
        # results.show()



        #byte_data = struct.unpack(const_unpack, message.depth.data)

        # Report detection result
        print(detections)
        print('TIME: ', str((time.time() - start)))

        for i in range(len(recent_positions)):
            msg = ObjectPositionResponse()
            msg.found = False
            recent_positions[i] = msg
        
        for i, detection in enumerate(detections):
            object_position_response = ObjectPositionResponse()
            object_position_response.found = True
        
            box = detections[i][:4]
            fov_x = 110
            fov_y = 70

            x_pos = int((box[0] + box[2]) / 2)
            y_pos = int((box[1] + box[3]) / 2)
            
            d_x = x_pos - (width / 2)
            d_y = y_pos - (height / 2)
            
            theta_x = (d_x * fov_x) / width
            theta_y = (d_y * fov_y) / height
            
            conf = detections[i][4]
            
            print(x_pos, y_pos, theta_x, theta_y)
           
            object_position_response.found      = True
            object_position_response.x_position = x_pos
            object_position_response.y_position = y_pos
            object_position_response.x_theta    = theta_x
            object_position_response.y_theta    = theta_y
            object_position_response.confidence = conf
        
        
            idx = int(detections[i][5])
            recent_positions[idx] = object_position_response
            print(idx)
            print(recent_positions[idx])
            recent_positions[idx] = object_position_response

            # draw bounding box on image
            cv2.rectangle(image_ocv, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 255, 255), 2)
            cv2.putText(image_ocv, f"{Targets(idx).name} {conf:.2f}", (int(box[0]), int(box[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255))

        print(f"processed in {time.time() - start}")

        msg = bridge.cv2_to_imgmsg(image_ocv, encoding='rgb8')
        PeriodicIO.bbox_pub.publish(msg)


        # THIS PRINTS a lOT
        # log("ml_node", "DEBUG", 'ml_node - done processing a frame in ' + str((time.time()-start)) + ' seconds')
    else:
        for i in range(len(Targets)):
            recent_positions[i] = None


def handle_obj_request(idx, msg):
    global latest_request_time
    latest_request_time = rospy.get_time()
    while recent_positions[idx] == None:
        rospy.sleep(0.005)
    return recent_positions[idx]

# Load the model
#model, classes, colors, output_layers = load_yolo()
model = load_yolo()
obj_pos_pub = None
bounding_pub = None
print("model loaded")

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

    for i in range(len(Targets)):
        recent_positions[i] = None
    latest_request_time = rospy.get_time() - TIME_THRESHOLD

    # Subscribe to the ZED left image and depth topics
    # For a full list of zed topics, see https://www.stereolabs.com/docs/ros/zed-node/#published-topics
    rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, zed_callback, queue_size=1)
    #rospy.Subscriber("/zed_nodelet/rgb/image_rect_color", Image, zed_callback, queue_size=1)

    PeriodicIO.bbox_pub = rospy.Publisher('/object_position/bbox', Image, queue_size=10)
    rospy.spin()

    cv2.destroyAllWindows()

