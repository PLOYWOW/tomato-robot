#!/usr/bin/env python3

import numpy as np
import os
import math
import time
import cv2
import struct
from numpy.core.numeric import NaN
import open3d as o3d
import matplotlib.pyplot as plt
import glob
import ctypes
import warnings


#ROS Required
import rospy
import ros_numpy
from std_msgs.msg import Header,String
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Image,CompressedImage
from std_msgs.msg import Bool,Float32MultiArray
from visualization_msgs.msg import Marker,MarkerArray
from tomato_detection.srv import SelectTomato,SelectTomatoResponse
from geometry_msgs.msg import Point,Pose,PoseStamped,PoseArray,PointStamped
from tf.transformations import quaternion_from_euler


#Yolor & Tensorrt
import torch
from utils.datasets import letterbox
from utils.general import non_max_suppression, scale_coords
from utils.plots import plot_one_box
from exec_backends.trt_loader import TrtModel

class YOLOR(object):
    def __init__(self,conf=0.7):
        max_size = 896
        ENGINE_PATH     = os.path.dirname(os.path.abspath(__file__))+'/engine/tomato_w6_fp16.trt'
        CLASS_NAME_PATH = os.path.dirname(os.path.abspath(__file__))+'/data/tomato.names'
        self.names = self.load_classes(CLASS_NAME_PATH)
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]
        self.imgsz = (max_size, max_size)
        # Load model
        self.conf = conf
        self.model = TrtModel(ENGINE_PATH, max_size,total_classes = len(self.names))
        self.bboxDict = dict((name,[]) for name in self.names)
        print(self.bboxDict)
    def load_classes(self,path):
    # Loads *.names file at 'path'
        with open(path, 'r') as f:
            names = f.read().split('\n')
        return list(filter(None, names))  # filter removes empty strings (such as last line)


    def resetDict(self,bboxDict):
        for k in bboxDict.keys():
            bboxDict[k] = []

    def detect(self, bgr_img):   
        self.resetDict(self.bboxDict)
        # Prediction
        ## Padded resize
        inp = letterbox(bgr_img, new_shape=self.imgsz, auto_size=64)[0]
        #inp = letterbox(bgr_img, new_shape=self.imgsz, auto_size=64,auto = False,scaleFill=True)[0]
        #print(inp.shape)
        inp = inp[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB
        inp = inp.astype('float32') / 255.0  # 0 - 255 to 0.0 - 1.0
        inp = np.expand_dims(inp, 0)
        #print(inp.shape,type(inp))        
        
        ## Inference
        t1 = time.time()
        pred = self.model.run(inp)[0]
        t2 = time.time()
        ## Apply NMS
        with torch.no_grad():
            pred = non_max_suppression(torch.tensor(pred), conf_thres=self.conf, iou_thres=0.6)
        t3 = time.time()
        print('Inference: {}'.format(t2-t1))
        print('NMS: {}'.format(t3-t2))
        #print('FPS: ', 1/(t3-t1))
    
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
                label_conf = '%s %.2f' % (self.names[int(cls)], conf)
                plot_one_box((x1, y1, x2, y2), visualize_img, label=label_conf, color=self.colors[int(cls)], line_thickness=3)
                label = self.names[int(cls)]
                self.bboxDict[label].append((int(x1.item()), int(y1.item()), int(x2.item()), int(y2.item())))

        #cv2.imwrite('./inference/output/trt_result.jpg', visualize_img)
        return visualize_img,self.bboxDict


class DepthImageHandler(object):
    def __init__(self):
        self.CAMINFO = {'topic': '/rgb/camera_info', 'msg': CameraInfo}
        self.COLOR = {'topic': '/rgb/image_raw', 'msg': Image}
        self.DEPTH = {'topic': '/depth_to_rgb/image_raw', 'msg': Image}
        self.isCamInfo = False
        self.PC = {'topic': '/hand_tomatoPC', 'msg': PointCloud2}
        
        self.H = 720
        self.W = 1280
        self.cropSize = 1 #1 for no crop
        self.header = Header() #Use for point cloud publisher
        self.fields = [PointField('x', 0, 7, 1), PointField('y', 4, 7, 1), PointField('z', 8, 7, 1), PointField('rgb', 16, 7, 1)]
        self.points = []
        self.pc = PointCloud2()
        

        self.color_image = np.empty((self.H, self.W ,3), dtype=np.uint8)
        self.depth_image = np.empty((self.H, self.W), dtype=np.uint16)
        self.aligned_image  = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask           = np.empty((self.H, self.W), dtype=np.bool)
        self.mask_image     = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask_depth     = np.empty((self.H, self.W), dtype=np.uint8)
        self.cropPointCloud = o3d.geometry.PointCloud() 

        self.camera_matrix = np.array([[0.0, 0, 0.0], [0, 0.0, 0.0], [0, 0, 1]], dtype=np.float32) 
        self.camera_distortion = np.array([0,0,0,0,0], dtype=np.float32)

        self.pickUV = [self.H//2,self.W//2]


        ENGINE_PATH     = os.path.dirname(os.path.abspath(__file__))+'/../yolor/engine/tomato_p6_fp16.trt'
        CLASS_NAME_PATH = os.path.dirname(os.path.abspath(__file__))+'/../yolor/data/tomato.names'
        self.model = YOLOR(conf=0.6)

        self.SCORE_THR = 0.85 #  cv2.getTrackbarPos('score','image')
        
        #create trackbar
        cv2.namedWindow('score_bar')
        cv2.createTrackbar('score','score_bar',1,100,self.nothing)
        cv2.setTrackbarPos('score','score_bar', int (self.SCORE_THR * 100) )
        cv2.createTrackbar('lh','score_bar',0,255,self.nothing)
        cv2.createTrackbar('ls','score_bar',0,255,self.nothing)
        cv2.createTrackbar('lv','score_bar',0,255,self.nothing)
        cv2.createTrackbar('uh','score_bar',0,255,self.nothing)
        cv2.createTrackbar('us','score_bar',0,255,self.nothing)
        cv2.createTrackbar('uv','score_bar',0,255,self.nothing)

        self.realTimeDetectionImage = np.zeros([self.H, self.W, 3], dtype=np.float)
        self.fullRedTomato = self.Tomato(stage = "b_fully_ripened")
        self.halfRedTomato = self.Tomato(stage = "b_half_ripened")
        self.greenTomato = self.Tomato(stage = "b_green")
        self.getLabelInstanceDict = {0:self.fullRedTomato ,
                                    1:self.halfRedTomato,
                                    2:self.greenTomato}


    class Tomato():
        def __init__(self,stage):
            self.stage = stage
            self.number = 0
            self.bboxes = []
            self.camPos = []
            self.hetaAction = [] # -1 rotate left,0 no rotate.1 rotate right
        def __str__(self):
            return f"tomato class : {self.stage}\nnumber :{self.number}"
        def getClosetTomatoPos(self,targetPos):
            if self.number == 0:
                return [0,0,0]
            Distances = [math.sqrt(sum((tomatoPos - targetPos) ** 2.0 for tomatoPos, targetPos in zip(tomatoPos, targetPos))) for tomatoPos in self.camPos]
            min_index = Distances.index(min(Distances))
            return self.camPos[min_index]
        def getClosetTomatoBBox(self,targetPos):
            if self.number == 0:
                return [0,0,0,0]
            Distances = [math.sqrt(sum((tomatoPos - targetPos) ** 2.0 for tomatoPos, targetPos in zip(tomatoPos, targetPos))) for tomatoPos in self.camPos]
            min_index = Distances.index(min(Distances))
            return self.bboxes[min_index]
        def reset(self):
            self.number = 0
            self.bboxes = []
            self.camPos = []
            self.hetaAction = []

    def nothing(self):
        pass


    def camInfoCallback(self, msg):
        self.header = msg.header
        self.K = msg.K
        self.width = msg.width  
        self.height = msg.height
        self.ppx = msg.K[2]
        self.ppy = msg.K[5]
        self.fx = msg.K[0]
        self.fy = msg.K[4] 
        
        self.cam_distortion_model = msg.distortion_model
        self.k1 = msg.D[0]
        self.k2 = msg.D[1]
        self.t1 = msg.D[2]
        self.t2 = msg.D[3]
        self.k3 = msg.D[4]
        self.isCamInfo = True
        self.camera_matrix = np.array([[self.fx, 0, self.ppx], [0, self.fy, self.ppy], [0, 0, 1]], dtype=np.float32) 
        self.camera_distortion = np.array([self.k1,self.k2,self.t1,self.t2,self.k3], dtype=np.float32)


    def colorCallback(self, msg):
        color_image = ros_numpy.numpify(msg)
        self.color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
        #self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)

    def depthCallback(self, msg):
        self.depth_image = ros_numpy.numpify(msg).astype(np.uint16)
    
    def pcCallback(self, msg):
        self.fields = msg.fields
        #print(self.fields)
    
    def publishImage(self,image):
        #### Create CompressedIamge ####
        #msg = Image()
        #msg.header.stamp = rospy.Time.now()
        #print(image.dtype)
        msg = ros_numpy.msgify(Image, image,encoding = "bgr8")
        # Publish new image
        self.image_pub.publish(msg)
        

    def find_nearest_nonzero(self,depth_img, target):
        #nonzero = cv2.findNonZero(depth_img)
        #distances = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 + (nonzero[:,:,1] - target[1]) ** 2)
        #nearest_index = np.argmin(distances)
        zeroPixel = depth_img[target[1],target[0]]
        count = 0
        while zeroPixel == 0:
            count += 1
            zeroPixel = depth_img[target[1],target[0]+count]
            if count > 5:
                break

        return target[0]+count,target[1]

    def depthToPoint3(self, depth_image, U, V):
        V =  np.clip(V,0,self.height-1)
        U =  np.clip(U,0,self.width-1)  
        
        x = (U - self.K[2]/self.cropSize)/self.K[0]
        y = (V - self.K[5]/self.cropSize)/self.K[4]     
        nearPixel = self.pixel_crop(depth_image,[5,5],(U,V))
        meanDepth = np.mean(nearPixel[np.nonzero(nearPixel)])
        if np.isnan(meanDepth):
            rospy.logwarn("Nan depth value, 0 mean depth is returned")
            meanDepth = 0
        #print(meanDepth,"mean")
        z = meanDepth / 1000
        x *= z
        y *= z
        # print(U,V,x,y,z)
        point3 = [x, y, z]
        return point3
        # Checks if a matrix is a valid rotation matrix.

    def pointToDepthUV(self, point3):
        [x,y,z] = point3  
        
        u = (x*self.K[0]/z) + self.K[2]/self.cropSize
        v = (y*self.K[4]/z) + self.K[5]/self.cropSize   
        return [int(u),int(v)]

    
    def rgbToFloat(self, rgb):
        return struct.unpack('f', struct.pack('i', rgb))[0]


    def depthToPoints(self, depth_image,color_image):
        [height, width] = depth_image.shape
        #print(depth_image.shape,color_image.shape)
        #print(self.K)
        nx = np.linspace(0, width-1, width)
        ny = np.linspace(0, height-1, height)
        u, v = np.meshgrid(nx, ny)
        #print(u)
        x = (u.flatten() - self.K[2]/self.cropSize)/self.K[0]
        y = (v.flatten() - self.K[5]/self.cropSize)/self.K[4]

        z = depth_image.flatten() /1000
        

        color_points = np.reshape(color_image, [-1,3])
        color_points = color_points[np.nonzero(z)]
        #rgb_int = [self.rgb2hex(q,w,e) for q,w,e in color_points[:]]
        
        #print(rgb_int)
        #bgr --> rgb
        r = color_points[:,2]
        g = color_points[:,1]
        b = color_points[:,0]
        rgb_int = 65536 * r   + 256 * g  + b
        rgb_int = [self.rgbToFloat(r) for r in rgb_int]
        
        #print(rgb_int)
        x = np.multiply(x,z)
        y = np.multiply(y,z)
        #print(np.nonzero(z))
        
        x = x[np.nonzero(z)]
        y = y[np.nonzero(z)]
        z = z[np.nonzero(z)]
        #print(z)
        r = r[np.nonzero(z)]
        g = g[np.nonzero(z)]
        b = b[np.nonzero(z)]
        #rgb_int = rgb_int[np.nonzero(z)]
        #rgb_int = [self.rgb2hex(x,y,z) for x in r for y in g for z in b]
        #print(len(x),len(y),len(z),len(r),len(g),len(b),len(rgb_int))
        points = np.stack((x,y,z,rgb_int), axis = -1) 
        #print(points)
        return points

    def isRotationMatrix(self,R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

   

    def boardCastMarkerFromCameraTF(self,markerPose):
        markerMessage = Float32MultiArray()
        markerMessage.data = markerPose.tolist()
        self.markerXYZRPY_pub.publish(markerMessage)
        #print(markerMessage)

    def findRotationFromTo(self,VecA,VecB):
        v = np.cross(VecA,VecB)
        s = np.linalg.norm(v)
        c = np.dot(VecA,VecB)
        I = np.identity(3)
        vXStr = '{} {} {}; {} {} {}; {} {} {}'.format(0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0)
        k = np.matrix(vXStr)
        r = I + k + np.matmul(k,k) * ((1 -c)/(s**2))
        return r

    def publishPoint3(self,pos3):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.header.frame_id

        point.point.x = pos3[0]
        point.point.y = pos3[1]
        point.point.z = pos3[2]
        
        self.point_pub.publish(point)
    
    def publishPointCloud(self, pc_pub, points):
        pc = point_cloud2.create_cloud(self.header, self.fields, points)
        self.cropPointCloud = pc
        pc_pub.publish(pc)


    def getTomatoMaskImageFromBbox(self,bbox):
        color_image = self.color_image
        x1,y1,x2,y2 = bbox
        masked_bbox = np.zeros_like(color_image)
        masked_bbox[y1:y2,x1:x2] = color_image[y1:y2,x1:x2]

        hsv = cv2.cvtColor(masked_bbox, cv2.COLOR_BGR2HSV)
        lh,ls,lv,uh,us,uv = 0,0,0,0,0,0
        l_h = cv2.getTrackbarPos("lh", "score_bar")
        l_s = cv2.getTrackbarPos("ls", "score_bar")
        l_v = cv2.getTrackbarPos("lv", "score_bar")

        u_h = cv2.getTrackbarPos("uh", "score_bar")
        u_s = cv2.getTrackbarPos("us", "score_bar")
        u_v = cv2.getTrackbarPos("uv", "score_bar")
        
        l_b = np.array([l_h, l_s, l_v])
        u_b = np.array([u_h, u_s, u_v])
        
        l_b = np.array([170, 70, 50])
        u_b = np.array([180, 255, 255])
        tomato_mask = cv2.inRange(hsv, l_b, u_b)
        return tomato_mask

    def pixel_crop(self,img, dim,pixel):
        width, height = img.shape[1], img.shape[0]
        # process crop width and height for max available dimension
        crop_width = dim[0] if dim[0]<img.shape[1] else img.shape[1]
        crop_height = dim[1] if dim[1]<img.shape[0] else img.shape[0] 
        mid_x, mid_y = int(pixel[0]), int(pixel[1])
        cw2, ch2 = int(crop_width/2), int(crop_height/2) 
        crop_img = img[mid_y-ch2:mid_y+ch2, mid_x-cw2:mid_x+cw2]
        return crop_img
    
    def checkHetaAvoidance(self,bbox):
        depth_image = self.depth_image
        isBlocked = 0
        avoidAction = 0
        x1,y1,x2,y2 = bbox
        boxSize = int(x2-x1)
        #print('boxSize',boxSize)
        tomatoMask = self.getTomatoMaskImageFromBbox(bbox)
        cX,cY = int((x2+x1)/2),int((y2+y1)/2)
        centerCheckBlock = self.pixel_crop(tomatoMask,(boxSize//4,boxSize//4),(cX,cY))
        isBlocked = 1 if np.count_nonzero(centerCheckBlock) < 0.9*centerCheckBlock.size else 0
        if isBlocked:
            leftCheckPixel = (x1-boxSize//2,cY)
            leftCheckBlock = self.pixel_crop(depth_image,(boxSize//4,boxSize//4),(leftCheckPixel))
            leftMeanNonZero = np.mean(leftCheckBlock[np.nonzero(leftCheckBlock)])
            rightCheckPixel = (x2+boxSize//2,cY)
            rightCheckBlock = self.pixel_crop(depth_image,(boxSize//4,boxSize//4),(rightCheckPixel))
            rightMeanNonZero = np.mean(rightCheckBlock[np.nonzero(rightCheckBlock)])
            print('leftMeanNonZero',leftMeanNonZero)
            print('rightMeanNonZero',rightMeanNonZero)
            avoidAction = -1 if leftMeanNonZero > rightMeanNonZero else 1

        avoidCheckBlock = self.pixel_crop(depth_image,(boxSize*2,boxSize*2),(cX,cY))
        cv2.imshow('center',centerCheckBlock)
        #cv2.imshow("avoider", avoidCheckBlock.astype('uint8') * 255)       
        return avoidAction 

    def publishTomatoPos3MarkerArray(self,tomatoPosList: list)-> None:
        tomatoMarkerArray = MarkerArray()
        for index,pos3 in enumerate(tomatoPosList):
            tomatoMarker = Marker()
            tomatoMarker.header.frame_id = self.header.frame_id # Camera Optical Frame
            tomatoMarker.header.stamp = rospy.Time.now()
            tomatoMarker.type = 2 # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            # Set the scale of the marker
            tomatoMarker.scale.x = 0.03
            tomatoMarker.scale.y = 0.03
            tomatoMarker.scale.z = 0.03
            # Set the color
            tomatoMarker.color.r = 0.0
            tomatoMarker.color.g = 1.0
            tomatoMarker.color.b = 0.0
            tomatoMarker.color.a = 1.0
            tomatoMarker.id = index
            tomatoMarker.pose.position.x = pos3[0]
            tomatoMarker.pose.position.y = pos3[1]
            tomatoMarker.pose.position.z = pos3[2]
            tomatoMarker.lifetime = rospy.Duration(0.1)
            tomatoMarkerArray.markers.append(tomatoMarker)
        self.array_pub.publish(tomatoMarkerArray)

    def publishTomatoPos3Array(self)-> None:
        tomatoObject = self.fullRedTomato
        tomatoPoseArray = PoseArray()
        tomatoPoseArray.header.frame_id = self.header.frame_id # Camera Optical Frame
        tomatoPoseArray.header.stamp = rospy.Time.now()
        rotationDict = {-1: quaternion_from_euler(0,math.radians(180),0),
                        0:quaternion_from_euler(0,math.radians(90),0),
                        1: quaternion_from_euler(0,0,0)}

        for index in range(tomatoObject.number):
            pos3 = tomatoObject.camPos[index]
            tomatoPose = Pose()
            tomatoPose.position.x = pos3[0]
            tomatoPose.position.y = pos3[1]
            tomatoPose.position.z = pos3[2]
            quat4 = rotationDict[tomatoObject.hetaAction[index]]
            tomatoPose.orientation.x = quat4[0]
            tomatoPose.orientation.y = quat4[1]
            tomatoPose.orientation.z = quat4[2]
            tomatoPose.orientation.w = quat4[3]
            #tomatoPose.orientation = rotationDict[tomatoObject.hetaAction[index]] #determine oreintation from tomato heta action
            tomatoPoseArray.poses.append(tomatoPose)
        self.array_pub.publish(tomatoPoseArray)
    
    def tomatoDetect(self,color_image,depth_image,isUpdateValue = True):
        
        if isUpdateValue:
            self.fullRedTomato.reset()
            self.halfRedTomato.reset()
            self.greenTomato.reset()
        detected_image,bboxDict = self.model.detect(color_image)
        
        screenPos = [(int((x2+x1)/2),int((y2+y1)/2)) for (x1,y1,x2,y2) in bboxDict["red_tomato"]]
        #print(f"screenNNN{screenPos}")
        camPos = [self.depthToPoint3(depth_image,cX,cY) for cX,cY in screenPos]
        hetaAction = [self.checkHetaAvoidance(bbox) for bbox in bboxDict["red_tomato"]]
        self.fullRedTomato.bboxes = bboxDict["red_tomato"]
        self.fullRedTomato.camPos = camPos
        self.fullRedTomato.number = len(self.fullRedTomato.camPos)
        self.fullRedTomato.hetaAction = hetaAction
        return detected_image

    def GetTomato(self):
        t1 = time.time()

        color_image, depth_image    = self.color_image, self.depth_image
        detected_image = self.tomatoDetect(color_image,depth_image)
        #print(self.fullRedTomato.bboxes,self.fullRedTomato.camPos)
        self.publishImage(detected_image)
        #self.publishTomatoPos3MarkerArray(self.fullRedTomato.camPos)
        closetBbox = self.fullRedTomato.getClosetTomatoBBox([0,0,0])
        masked_tomato = self.getTomatoMaskImageFromBbox(closetBbox)
        print(self.fullRedTomato.hetaAction)
        self.publishTomatoPos3Array()

        t2 = time.time()
        FPS = f"FPS:{1/(t2-t1)}"
        cv2.putText(detected_image, FPS, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_4)
        cv2.imshow('frame', detected_image)
        cv2.imshow('masked', masked_tomato)
        cv2.waitKey(1)

    def process(self):
        rospy.init_node('markerFinder', anonymous=True)
        #r = rospy.Rate(60)
        rospy.Subscriber(self.CAMINFO['topic'], self.CAMINFO['msg'], self.camInfoCallback)
        rospy.Subscriber(self.COLOR['topic'], self.COLOR['msg'], self.colorCallback)
        rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['msg'], self.depthCallback)
        #rospy.Subscriber(self.PC['topic'],    self.PC['msg'],    self.pcCallback)
    
        ###publisher
        self.tomatoPC_pub = rospy.Publisher('/hand_tomatoPC', PointCloud2, queue_size=1)

        self.point_pub = rospy.Publisher('/hand_tomato_pick_pos', PointStamped, queue_size=1)
        
        self.array_pub = rospy.Publisher('/tomatoArray', PoseArray, queue_size=1)

        self.image_pub = rospy.Publisher('/detectedImage', Image, queue_size=1)
                        

        r = rospy.Rate(60) # 10hz 
        while not rospy.is_shutdown():
                if self.isCamInfo:
                    t1 = time.time()
                    self.GetTomato()
                    t2 = time.time()
                    r.sleep()
                    #print(f"TPF:{t2-t1}second\nFPS:{1/(t2-t1)}")


         
if __name__ == '__main__':
    try:
        _depthImageHandler = DepthImageHandler()
        _depthImageHandler.process()

    except rospy.ROSInterruptException:
        pass
