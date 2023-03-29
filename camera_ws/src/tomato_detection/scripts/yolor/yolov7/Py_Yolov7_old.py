import matplotlib.pyplot as plt
import torch
import cv2
#from torchvision import transforms
import numpy as np
from utils.datasets import letterbox
from utils.general import non_max_suppression_kpt
from utils.plots import output_to_keypoint, plot_skeleton_kpts
from numpy import random
from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel
from torch.cuda import amp,empty_cache
import time





# Give path of source image.
class YOLOV7(object):
    '''
                model_weights = "runs/train/exp39/weights/last.pt",
                yaml          = "Trash-5/data.yaml",
                imgsize = 640, # default image size
                confthres= 0.1, # confidence threshold for inference.
                iouthres = 0.45, # NMS IoU threshold for inference.
                device = '0',  # device to run our model i.e. 0 or 0,1,2,3 or cpu
                classes = None  # list of classes to filter or None 
    '''
    def __init__(self,
                model_weights ,
                yaml          ,
                imgsize , # default image size
                confthres, # confidence threshold for inference.
                iouthres , # NMS IoU threshold for inference.
                device ,  # device to run our model i.e. 0 or 0,1,2,3 or cpu
                classes , # list of classes to filter or None 
                ) :

        self.model_weights = model_weights
        self.yaml          = yaml
        self.imgsize = imgsize # default image size
        self.confthres= confthres # confidence threshold for inference.
        self.iouthres = iouthres # NMS IoU threshold for inference.
        self.device = device  # device to run our model i.e. 0 or 0,1,2,3 or cpu
        self.classes = classes # list of classes to filter or None 
        
        self.get_xywh_point = []
        self.get_pred_point = []

        

        self.opt =  {
    
                    "weights": model_weights, # Path to weights file default weights are for nano model
                    "yaml"   : yaml,
                    "img-size": imgsize, # default image size
                    "conf-thres": confthres, # confidence threshold for inference.
                    "iou-thres" : iouthres, # NMS IoU threshold for inference.
                    "device" : device,  # device to run our model i.e. 0 or 0,1,2,3 or cpu
                    "classes" : classes  # list of classes to filter or None

                    }
    
        
    
    


    def letterbox(self,img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        # Resize and pad image while meeting stride-multiple constraints
        
        shape = img.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better test mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        
        #print("top, bottom, left, right", top, bottom, left, right)
        return img, ratio, (dw, dh)

    def load_model(self):
        print("Loading model............................................")
        with torch.no_grad():
            weights, imgsz = self.opt['weights'], self.opt['img-size']
            set_logging()
            device = select_device(self.opt['device'])
            half = device.type != 'cpu'
            model = attempt_load(weights, map_location=device)  # load FP32 model
            stride = int(model.stride.max())  # model stride
            imgsz = check_img_size(imgsz, s=stride)  # check img_size
            if half:
                model.half()  # to FP16
            names = model.module.names if hasattr(model, 'module') else model.names  # get class names
            #print(model)
            print("Model loaded in FP16 mode")
            #names = model.module.names if hasattr(model, 'module') else model.names
            #colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]
            colors = [[0, 0, 200], [200,0,0],[0, 200, 0]]
            
            # Run inference
            if device.type != 'cpu':

                model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
                print("Model loaded successfully")
            # Apply NMS

            classes = None
            if self.opt['classes']:
                classes = []
                for class_name in self.opt['classes']:
                    classes.append(self.opt['classes'].index(class_name))

        

        self.model = model
        self.device = device
        self.half = half
        self.names = names
        self.colors = colors
        self.stride = stride
        self.imgsz = imgsz
        self.classes = classes

    def resetDict(self,bboxDict):
        for k in bboxDict.keys():
            bboxDict[k] = []

        
    def detect(self, img0):
        #print("Detect##################")
        
        #print("###############################",self.names)
        self.bboxDict = dict((name,[]) for name in self.names)
        
        self.resetDict(self.bboxDict)
        #empty_cache()

        
        #img = cv2.imread(img_path)
        # Padded resize
        img = img0
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        #img = self.letterbox(img, new_shape=self.imgsz)[0]
        # Convert
        #img = img[:, :, ::-1].transpose(2, 0, 1)
        img = letterbox(img, self.imgsz, stride=self.stride)[0]
        #img = letterbox(frame, imgsz, stride=stride)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = self.model(img, augment= False)[0]

        


        pred = non_max_suppression(pred, self.opt['conf-thres'], self.opt['iou-thres'], classes= self.classes, agnostic= False)
        t2 = time_synchronized()
        for i, det in enumerate(pred):
            s = ''
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(img0.shape)[[1, 0, 1, 0]]
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
            
            #for *xyxy, conf, cls in reversed(det):
            for  x1, y1, x2, y2, conf, cls in reversed(det):
                '''

                label = f'{self.names[int(cls)]} {conf:.2f}'
                #label = f'{conf:.2f}'
                plot_one_box(xyxy, img0, label=label, color=self.colors[int(cls)], line_thickness=2)
                #print(self.colors[int(0)])
                #xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)))
                new = np.array(xywh)
                #print(label, new[0])
                self.get_pred_point = label
                self.get_xywh_point = [new[0][0],new[0][1],new[0][2],new[0][3]]
                #print(xyxy)
                #print(label)
                cv2.circle(img0, (int(new[0][0]), int(new[0][1])), 5, (0, 0, 255), -1)
                
                '''
                label_conf = '%s %.2f' % (self.names[int(cls)], conf)
                plot_one_box((x1, y1, x2, y2), img0, label=label_conf, color=self.colors[int(cls)], line_thickness=3)
                label = self.names[int(cls)]
                self.bboxDict[label].append((int(x1.item()), int(y1.item()), int(x2.item()), int(y2.item())))
                cv2.circle(img0, (int(x1.item()),int(y1.item())), 5, (255,255,0), -1)
                cv2.circle(img0, (int(x2.item()),int(y2.item())), 5, (255,255,0), -1)

                left = int(x1.item())
                right = int(x2.item())
                top = int(y1.item())
                bottom = int(y2.item())

                i_centor = int(left + right) / 2
                j_centor = int(top + bottom) / 2

                cv2.circle(img0, (int(i_centor),int(j_centor)), 14, (115,100,0), -1)

                #print("bbox",self.bboxDict)
                

                #print(label_conf)
        return img0

    def detect_(self,img_in):
        
        self.img_in = img_in
  
       
        #cap = cv2.VideoCapture("testAll.mp4")
        #cap = cv2.VideoCapture(0)
    
        with torch.no_grad():
                
                empty_cache()
                #ret, frame = cap.read()
                frame = self.img_in
                #new_frame_time = time.time()
                #print(frame.shape)

                #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                #frame = cv2.resize(frame, [1280,720])
                #print("Shape ==>> ",frame.shape)
                #frame = np.array(frame).astype(np.float16) / 255.0
                        

                img = self.detect(frame)
                #point = self.get_xywh_point
                #pred = self.get_pred_point
                #print(pred,point)

                #fps = 1/(new_frame_time-prev_frame_time)
                #prev_frame_time = new_frame_time
                #fps = int(fps)
                #fps = str(fps)
                #font = cv2.FONT_HERSHEY_SIMPLEX
                #cv2.putText(img, fps, (7, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA)  
                
                #print(fps) 
                
                                
                #cv2.imshow("Image", img)
                #cv2.imshow("Image", frame)
                    
        return img,self.bboxDict

        #cap.release()
cv2.destroyAllWindows()       
    
'''   
# Model
model_weights = "runs/train/exp35/weights/best.pt"
yaml          = "tomatoData/data.yaml"
imgsize = 640 # default image size
confthres= 0.1 # confidence threshold for inference.
iouthres = 0.45 # NMS IoU threshold for inference.
device = '0'  # device to run our model i.e. 0 or 0,1,2,3 or cpu
classes = [] #["Grean_tomato"]  # list of classes to filter or None     
yolo = YOLOV7(model_weights,yaml,imgsize,confthres,iouthres,device,classes)


print("Running............................................")
yolo.load_model()
#cap = cv2.VideoCapture("sample.mp4")
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
        

    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #frame = cv2.resize(frame, [640,640])
            
    img = yolo.detect(frame)
    p = yolo.get_point
    print(p)
                
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

'''

