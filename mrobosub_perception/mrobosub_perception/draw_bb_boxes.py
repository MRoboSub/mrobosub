#!/usr/bin/env python
import cv2
import enum
import torch
import os

class Targets(enum.Enum):
    ABYDOS = 0
    EARTH = 1
    TAURUS = 2
    SERPENS_CAPUT = 3
    AURIGA = 4
    CETUS = 5

def load_yolo():
    # load model
    if torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")

    path = os.path.abspath(os.path.join(os.path.dirname(__file__)))
    yolo_path = os.path.join(path, 'yolov5')

    model_path = os.path.join(path, "models/model_2a.pt")
    model = torch.hub.load(yolo_path, 'custom', path=model_path, source='local')  # local repo
    model.conf = 0.25  # NMS confidence threshold
    return model

WIDTH = 672
HEIGHT = 376

model = load_yolo()


for i in range(1, 5):
    fname = f"/home/mrobosub/img{i}"
    img = cv2.imread(fname)
    img = cv2.resize(img, (WIDTH, HEIGHT))

    outputs = model(img, size=WIDTH)
    detections = outputs.xyxy[0].cpu().numpy()   # get the detections

    for i, detection in enumerate(detections):
        box = detections[i][:4]
        conf = detections[i][4]

        idx = int(detections[i][5])

        # draw bounding box on image
        cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 0, 0), 2)
        cv2.putText(img, f"{Targets(idx).name} {conf:.2f}", (int(box[0]), int(box[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0))

    cv2.imwrite(f"{fname}_bbox.png", img)
