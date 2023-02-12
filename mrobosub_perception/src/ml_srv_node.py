#!/usr/bin/env python

from functools import partial

import rospy
from ObjectPosition.srv import ObjectPosition, ObjectPositionResponse
import cv2
from zed_interfaces.msg import RGBDSensors
from cv_bridge import CvBridge
import numpy as np 
import time
import sys
import enum
from rsub_log import log
from get_depth import get_avg_depth
import struct
from sensor_msgs.msg import Image

height = 376
width = 1344
bridge = CvBridge()
const_unpack = ''
for i in range(height*width*2):
    const_unpack += 'B'


log("ml_node", "INFO", "Starting")
log("ml_node", "DEBUG", 'python version: ' + sys.version)
log("ml_node", "DEBUG", 'cv2 version: ' + cv2.__version__)
log("ml_node", "DEBUG", 'cv2 location:'+ cv2.__file__)

CONFIDENCE = 0.9
TIME_THRESHOLD


class Targets(enum.Enum):
    GATE = 0
    GMAN = 1
    BOOTLEGGER = 2
    GUN = 3
    BADGE = 4
    COUNT = 5

recent_positions = []
latest_request_time = None

def load_yolo():
    ''' Load pretrained weights, initialize CUDA. '''
    net = cv2.dnn.readNet("/home/jetson/catkin_ws/src/models/best.onnx")

    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

    layers_names = net.getLayerNames()
    #this should really only be one layer.
    output_layers = [layers_names[i-1] for i in net.getUnconnectedOutLayers()]

    return net, output_layers


def detect_objects(img):
    ''' get all detections from image using detection network. should return 25200 detections. '''
    global model
    global output_layers

    blob = cv2.dnn.blobFromImage(img, scalefactor=0.00392, size=(640, 640), mean=(0, 0, 0), swapRB=True, crop=False)
    model.setInput(blob)
    outputs = model.forward(output_layers)
    return outputs


def get_box_dimensions(outputs, height, width): 
    ''' Iterate through all detections, choose detections over confidence threshold. '''
    # class_ids = []
    # confidences = []
    # boxes = []
    # rows = outputs[0].shape[1]

    outputs = np.array(outputs)
    # filter by box confidence threshold
    filtered = outputs[0,0,(outputs[0,0,:,4] > 0.2)]

    # filter by class confidence threshold
    maxes = np.amax(filtered[:,5:],axis=1)
    max_filter = np.where(maxes > 0.25, True, False)
    filtered = filtered[(max_filter)]

    class_ids = np.argmax(filtered[:,5:],axis=1)
    confidences = np.amax(filtered[:,5:],axis=1)
    boxes = np.zeros((len(filtered),4))
    for i, row in enumerate(filtered):
        x,y,w,h = row[:4]
        left = int(x - 0.5 * w)
        top = int(y - 0.5 * h)
        width = int(w)
        height = int(h)
        boxes[i] = np.array([left,top,width,height])
    
    '''
    for r in range(rows):
        row = outputs[0][0][r]
        confidence = row[4]
        #print('BOX CONF',confidence)
        # This value is the box confidence. confidence that any object exists in the box.
	if confidence >= 0.2:
            classes_scores = row[5:]
            _,_,_, max_indx = cv2.minMaxLoc(classes_scores)
            class_id = max_indx[1]
	    # this  value is the class confidence,  confidence that the box contains this class.
            if(classes_scores[class_id] > 0.25):
                confidences.append(confidence)
                #print('found :' + classes[class_id])
		class_ids.append(class_id)
                x,y,w,h = row[0], row[1], row[2], row[3]
                left = int(x - 0.5 * w)
                top = int(y - 0.5 * h)
                width = int(w)
                height = int(h)
                box = np.array([left,top,width,height])
                boxes.append(box)
    '''

    # Perfom NMSBoxes algorithm, merges overlapping bounding boxes that have the same prediction to create an average bounding box.
    indexes = cv2.dnn.NMSBoxes(boxes, confidences,0.25,0.45)
    indexes = list(indexes)
    
    result_class_ids = np.zeros(len(indexes))
    result_confidences = np.zeros(len(indexes))
    result_boxes = np.zeros(len(indexes))

    if len(indexes) > 0:
        result_confidences = (confidences[indexes])
        result_class_ids = (class_ids[indexes])
        result_boxes = (boxes[indexes])


    return result_class_ids, result_confidences, result_boxes


def draw_labels(boxes, confs, class_ids, img): 
	# NON FUNCTIONING: Draw bounding boxes on input image and show.
	indexes = cv2.dnn.NMSBoxes(boxes, confs, 0.5, 0.4)
	font = cv2.FONT_HERSHEY_PLAIN
	for i in range(len(boxes)):
		if i in indexes:
			x, y, w, h = boxes[i]
			print('(x, y, w, h):', boxes[i])
			#label = str(classes[class_ids[i]])
			#color = colors[0] #color = colors[i]
			#cv2.rectangle(img, (x,y), (x+w, y+h), color, 2)
			#cv2.putText(img, label, (x, y - 5), font, 1, color, 1)
	cv2.imshow("Image", img)


def gray_world(img):
    # Perform white-balancing on input image.
    def whitebalance(channel, perc = 0.05):
        minimum, maximum = (np.percentile(channel, perc), np.percentile(channel, 100.0-perc)) 
        channel = np.uint8(np.clip((channel-minimum)*255.0/(maximum-minimum), 0, 255))
        return channel

    return np.dstack([whitebalance(channel, 0.05) for channel in cv2.split(img)])


def zed_callback(message):
    global img_seen_num, img_num
    if rospy.get_time() - latest_request_time < TIME_THRESHOLD:
        
        # print("zed callback")
        # Convert the zed image to an opencv image
        bridge = CvBridge()
        image_ocv = bridge.imgmsg_to_cv2(message.rgb, desired_encoding='passthrough')

        # Remove the 4th channel (transparency)
        image_ocv = image_ocv[:,:,:3]

        # Perform gray world assumption on image
        #image_ocv = gray_world(image_ocv)

        # Find any objects in the image
        height, width, channels = image_ocv.shape   # shape of the image
        outputs = detect_objects(image_ocv)   # get raw detection data
        class_ids, confs, boxes = get_box_dimensions(outputs, height, width)
        # log("ml_node", "DEBUG", 'ml_node - ' + str(len(class_ids)) + ' detections')

        # Draw any bounding boxes and display the image
        #draw_labels(boxes, confs, class_ids, image_ocv)

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

        # THIS PRINTS a lOT
        # log("ml_node", "DEBUG", 'ml_node - done processing a frame in ' + str((time.time()-start)) + ' seconds')
    else:
        for i in range(Targets.COUNT):
            recent_positions[i] = None
        

def handle_obj_request(idx, msg):
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
    mk_service = lambda name, idx: rospy.Service(f'object_position/{name}', ObjectPosition, lambda msg : handle_obj_request(idx, msg))
    gate_srv = mk_service('gate',Targets.GATE)
    gman_srv = mk_service('gman',Targets.GMAN)
    bootlegger_srv = mk_service('bootlegger',Targets.BOOTLEGGER)
    gun_srv = mk_service('gun',Targets.GUN)
    badge_srv = mk_service('badge',Targets.BADGE)

    for i in range(Targets.COUNT):
        recent_positions[i] = None
    latest_request_time = rospy.get_time() - TIME_THRESHOLD

    # Subscribe to the ZED left image and depth topics
    # For a full list of zed topics, see https://www.stereolabs.com/docs/ros/zed-node/#published-topics
    rospy.Subscriber("/sync_nodelet/rgbd_sens", RGBDSensors, zed_callback, queue_size=1)
    #rospy.Subscriber("/zed_nodelet/rgb/image_rect_color", Image, zed_callback, queue_size=1)   
    rospy.spin()
    
    cv2.destroyAllWindows()

