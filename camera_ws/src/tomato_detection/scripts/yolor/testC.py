from yolov7.Py_Yolov7 import YOLOV7
import cv2
import time
from torch.cuda import amp,empty_cache
import numpy as np
import torch
print("Clear cache")
# Model
model_weights = "final_model/exp25/weights/epoch_2499.pt"
yaml          = "Data0/data.yaml"
#model_weights = "runs/train/exp/weights/best.pt"
#yaml          = "Trash-5/data.yaml"
imgsize = 640 # default image size
confthres= 0.80 # confidence threshold for inference.
iouthres = 0.45 # NMS IoU threshold for inference.
device = '0'  # device to run our model i.e. 0 or 0,1,2,3 or cpu
classes = []  #["Grean_tomato"]  # list of classes to filter or None     

#yolo = YOLOV7(model_weights,yaml,imgsize,confthres,iouthres,device,classes)

yolo = YOLOV7()
print("Running............................................")
yolo.load_model()
cap = cv2.VideoCapture("testAll.mp4")
#cap = cv2.VideoCapture(0)
prev_frame_time = 0
new_frame_time = 0

while True:
        ret, frame = cap.read()
        frame = cv2.resize(frame, [1280,720])
        new_frame_time = time.time()
            

        #cv2.imshow("Image",frame)
        #img,Box = yolo.detect_(frame)
        #fps = 1/(new_frame_time-prev_frame_time)
        #prev_frame_time = new_frame_time
        #fps = int(fps)
        #fps = str(fps)
        #font = cv2.FONT_HERSHEY_SIMPLEX
        #cv2.putText(img, fps, (7, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA)  
        

        cv2.imshow("Image", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
