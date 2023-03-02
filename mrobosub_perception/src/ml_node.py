#import rospy
import argparse
import sys
import os
import time

# ZED IMPORTS
#sofrom zed_interfaces.msg import RGBDSensors

# Inference imports
import torch
import numpy as np

# init device variable
global device, model

def load_model():
    # load model
    #device = torch.device("cuda:0")
    if torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")
    model = torch.hub.load('./yolov5', 'custom', path='./models/best.pt', source='local')  # local repo
    model.conf = 0.25  # NMS confidence threshold
      iou = 0.45  # NMS IoU threshold
      agnostic = False  # NMS class-agnostic
      multi_label = False  # NMS multiple labels per box
      classes = None  # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
      max_det = 1000  # maximum number of detections per image
      amp = False  # Automatic Mixed Precision (AMP) inference
    return model

def inference(message, model):
    start = time.time()
    global img_seen_num, img_num
    # load image
    # convert the zed image to a numpy array
    img_np = np.asarray(message.rgb)
    img = torch.from_numpy(img_np).to(device)
    im = img.float() / 255.0
    pred = model(im)
    
    seen = 0
    # inference - for loop over predictions
    for i, det in enumerate(pred):
        seen+=1
    
    # return bbox as ros message


if __name__ == '__main__':
    # initialize the node
    #rospy.init_node('ml_node', anonymous=False)
    print("ml_node initialized")
    
    # initialize the model
    load_model()
    
    # subscribe to the zed topic and call the inference function
    #rospy.Subscriber("/sync_nodelet/rgbd_sens", RGBDSensors, inference, queue_size=1)
    
    #rospy.spin()

    pass
    