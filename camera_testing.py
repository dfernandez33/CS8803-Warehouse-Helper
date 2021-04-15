## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import numpy as np
import cv2
from camera_manager import CameraManager
from pyzbar import pyzbar
import pyrealsense2 as rs
import time
import jetson.inference
import jetson.utils
import json

COCO_INSTANCE_CATEGORY_NAMES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]
    
colorizer = rs.colorizer()
cam_manager = CameraManager()
model = jetson.inference.detectNet('ssd-mobilenet-v2', threshold=0.4)

time.sleep(2)

while True:
    start_time = time.time()
    rgb_frame, depth_frame = cam_manager.get_frames()
    detections = model.Detect(jetson.utils.cudaFromNumpy(rgb_frame))
    for detection in detections:
        text = "{} {}%".format(COCO_INSTANCE_CATEGORY_NAMES[detection.ClassID], detection.Confidence)
        print(text)

    barcodes = pyzbar.decode(rgb_frame)
    # loop over the detected barcodes
    for barcode in barcodes:
        # extract the bounding box location of the barcode and draw the
        # bounding box surrounding the barcode on the image
        (x, y, w, h) = barcode.rect
        # the barcode data is a bytes object so if we want to draw it on
        # our output image we need to convert it to a string first
        barcodeData = json.loads(barcode.data.decode("utf-8"))
        barcodeType = barcode.type
        # draw the barcode data and barcode type on the image
        depth = depth_frame[x:x+w, y:y+h].astype(float)
        depth = depth * cam_manager.depth_scale
        dist,_,_,_ = cv2.mean(depth[depth > 0])
        barcodeData['distance'] = dist
        # print the barcode type and data to the terminal
        print("[INFO] Found {} barcode: {} at {} meters".format(barcodeType, barcodeData, dist))

    print("Time to process frame: {}s (FPS {})".format(time.time() - start_time, model.GetNetworkFPS()))

