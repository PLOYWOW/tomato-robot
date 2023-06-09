import argparse
import os
import platform
import shutil
import time
from pathlib import Path

import cv2
import torch
import numpy as np
from numpy import random

from utils.datasets import letterbox
from utils.general import non_max_suppression, scale_coords
from utils.plots import plot_one_box
import matplotlib.pyplot as plt
import onnxruntime as rt
# from models.models import Darknet

def load_classes(path):
    # Loads *.names file at 'path'
    with open(path, 'r') as f:
        names = f.read().split('\n')
    return list(filter(None, names))  # filter removes empty strings (such as last line)

class YOLOR(object):
    def __init__(self, 
            model_weights = './onnx/yolor_p6.onnx', 
            imgsz = (640, 640), 
            names = 'data/coco.names'):
        self.names = load_classes(names)
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]
        self.imgsz = imgsz
        # Load model
        self.model = rt.InferenceSession(model_weights)


    def detect(self, bgr_img, threshold = 0.1):   
        # Prediction
        ## Padded resize
        inp = letterbox(bgr_img, new_shape=self.imgsz, auto_size=64,auto = False,scaleFill=True)[0]
        # print(inp.shape)
        # inp = cv2.resize(bgr_img, self.imgsz)
        inp = inp[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB
        inp = inp.astype('float32') / 255.0  # 0 - 255 to 0.0 - 1.0
        inp = np.expand_dims(inp, 0)
        # inp = np.transpose(inp, (0, 3, 1, 2))
        print(inp.shape)
        ## Convert to torch
        
        
        ## Inference
        t1 = time.time()
        ort_inputs = {self.model.get_inputs()[0].name: inp}
        pred = self.model.run(None, ort_inputs)[0]
        t2 = time.time()
        ## Apply NMS
        with torch.no_grad():
            pred = non_max_suppression(torch.tensor(pred), conf_thres=threshold, iou_thres=0.6)
        t3 = time.time()
        print('Inference: {}'.format(t2-t1))
        print('NMS: {}'.format(t3-t2))
    
        # Process detections
        visualize_img = bgr_img.copy()
        det = pred[0]  # detections per image
        if det is not None and len(det):
            # Rescale boxes from img_size to im0 size
            _, _, height, width = inp.shape
            h, w, _ = bgr_img.shape
            det[:, 0] *= w/width
            det[:, 1] *= h/height
            det[:, 2] *= w/width
            det[:, 3] *= h/height
            for x1, y1, x2, y2, conf, cls in det:       # x1, y1, x2, y2 in pixel format
                label = '%s %.2f' % (self.names[int(cls)], conf)
                plot_one_box((x1, y1, x2, y2), visualize_img, label=label, color=self.colors[int(cls)], line_thickness=3)

        #cv2.imwrite('./inference/output/result.jpg', visualize_img)
        return visualize_img

if __name__ == '__main__':
    model = YOLOR()
    #cap = cv2.VideoCapture("inference/images/tomato_wakamatsu.mp4")
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('inference/output/trt_result.avi', fourcc, 20.0, (640,  480))   
    cap = cv2.VideoCapture(6)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    fps = 0.0
    t1 = time.time()
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        #frame = cv2.flip(frame, 0)
        print(frame.shape,"frame")
        detected_image = model.detect(frame)
        t2 = time.time()
        curr_fps = 1.0 / (t2 - t1)
        fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
        FPS = f"FPS:{fps}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(detected_image, FPS, (50, 50), font, 1, (0, 0, 0), 2, cv2.LINE_4)
        cv2.imshow('frame', detected_image)
        t1 = t2
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # After the loop release the cap object
    cap.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

