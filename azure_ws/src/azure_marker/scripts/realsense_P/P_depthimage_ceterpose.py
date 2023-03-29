#!/usr/bin/env python


import numpy as np
import os
import math
import time
import cv2
import json
import mmcv
import struct
from cv_bridge import CvBridge
bridge = CvBridge()
from mmdet.apis import inference_detector, init_detector

import pycocotools.mask as maskUtils
from pycocotools.coco import COCO as coco

import rospy
import ros_numpy
from std_msgs.msg import Header,String
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Image
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker,MarkerArray
from playsound import playsound

import glob
import pyrealsense2 as rs
import torch
from geometry_msgs.msg import Point,Pose,PoseStamped,PoseArray
import ipdb
import tifffile as tiff
import open3d as o3d
import ctypes



import vlc
last_press = 0
# ================================================================================ CHANGED ============
np.set_printoptions(threshold=np.inf)
# ================================================================================ CHANGED ============


def find_latest_file(path):
    files = os.listdir(path)
    paths = [os.path.join(path, basename) for basename in files]
    return max(paths, key=os.path.getctime)


def send_traj_point_marker(marker_pub, pose, id, rgba_tuple):
    marker = Marker()
    marker.header.frame_id = "/camera_color_optical_frame"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "traj_point" 
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.r = rgba_tuple[0]
    marker.color.g = rgba_tuple[1]
    marker.color.b = rgba_tuple[2]
    marker.color.a = rgba_tuple[3]
    marker.lifetime = rospy.Duration(0.3,0)
    marker_pub.publish(marker)
def send_pose_stamp(pose_pub,pose):
    transformedPoint = Pose()
    

class  dist(object):
    def __init__(self,depthpoint,DIST):
        if(depthpoint is not None):
            self.X          = depthpoint[0]    
            self.Y          = depthpoint[1]  
            self.Z          = depthpoint[2]   
        if(DIST is not None):
            self.distance   = DIST
    first      = 1



class PointcloudMasking(object):
    def __init__(self):
        self.CAMINFO = {'topic': '/camera/color/camera_info', 'msg': CameraInfo}
        self.COLOR = {'topic': '/camera/color/image_raw', 'msg': Image}
        self.DEPTH = {'topic': '/camera/aligned_depth_to_color/image_raw', 'msg': Image}
        self.PC = {'topic': '/camera/depth/color/points', 'msg': PointCloud2}
        
          

        ###publisher&subscriber###
        self.pc_pub_spaghetti = rospy.Publisher('/pointcloud_spaghetti', PointCloud2, queue_size=1)
        self.cloud_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2,self.subPointCloudCallback,queue_size=1, buff_size=52428800)
        self.snap_sub = rospy.Subscriber("/snap", Bool,self.snapCallback,queue_size=10)
        ###############

        self.H = 720
        self.W = 1280
        self.header = Header()
        self.fields = [PointField('x', 0, 7, 1), PointField('y', 4, 7, 1), PointField('z', 8, 7, 1), PointField('rgb', 16, 7, 1)]
        self.points = []
        self.pc = PointCloud2()

        self.color_image = np.empty((self.H, self.W ,3), dtype=np.uint8)
        self.depth_image = np.empty((self.H, self.W), dtype=np.uint16)
        self.aligned_image  = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask           = np.empty((self.H, self.W), dtype=np.bool)
        self.mask_image     = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask_depth     = np.empty((self.H, self.W), dtype=np.uint8)
        self.fullPointCloud = o3d.geometry.PointCloud() 
        self.spaghettiPointCloud = o3d.geometry.PointCloud() 
        self.snapMessage = False
        self.sameSnap = 1

        # PATH_TO_PROJECT     = os.path.dirname(os.path.abspath(__file__))+'/mmdetection/karaage-onigiri-box-chip'
        # ANNOTATION_FILENAME = 'trainval.json'
        # CHECKPOINT_FILENAME = 'epoch_5000.pth'
        # CONFIG_FILENAME     = 'cascade_mask_rcnn_r50_fpn_1x.py'
        PATH_TO_PROJECT     = os.path.dirname(os.path.abspath(__file__))+'/mmdetection/newbox_food'
        ANNOTATION_FILENAME = 'trainval.json'
        CHECKPOINT_FILENAME = 'epoch_14000.pth'
        CONFIG_FILENAME     = 'tsuji_htc_newbox_food.py'

        # PATH_TO_PROJECT     = os.path.dirname(os.path.abspath(__file__))+'/mmdetection/noodles'
        # ANNOTATION_FILENAME = 'bologna.json'
        # CHECKPOINT_FILENAME = 'bologna.pth'
        # CONFIG_FILENAME     = 'cascade_mask_rcnn_r50_fpn_1x_6classses.py'


        annotation_file     = os.path.join(PATH_TO_PROJECT, ANNOTATION_FILENAME)
        json_file           = open(annotation_file)
        coco                = json.load(json_file)
        checkpoint_file     = os.path.join(PATH_TO_PROJECT, CHECKPOINT_FILENAME)
        config_file         = os.path.join(PATH_TO_PROJECT, CONFIG_FILENAME)
        self.class_names    = [category['name'] for category in coco['categories']] 
        # print(self.class_names)
        # ipdb.set_trace()
        self.modelDet          = init_detector(config_file, checkpoint_file, device=torch.device('cuda', 0))
        self.class_names.append('reserved1')
        self.modelDet.CLASSES  = self.class_names
        self.SCORE_THR = 0.95 #  cv2.getTrackbarPos('score','image')
        
        #create trackbar
        cv2.namedWindow('score_bar')
        cv2.createTrackbar('score','score_bar',1,100,self.nothing)
        cv2.setTrackbarPos('score','score_bar', int (self.SCORE_THR * 100) )
        
         ##FOR AR AR AR####
        self.objectPose = {
            'karaage': Pose() ,'onigiri':Pose()  ,'box':Pose()   ,'bologna':Pose() ,'spaghetti': Pose() ,'area1': Pose() ,'area2': Pose()  ,'area3': Pose()
        }

        self.objectDictID = {
                0:'karaage',1:'onigiri',2:'box',3:'bologna',4:'spaghetti',5:'area1',6:'area2',7:'area3'
        }

        self.getFoodID = {
                'karaage':0,'onigiri':1,'box':2,'bologna':3,'spaghetti':4,'area1':5,'area2':6,'area3':7
        }

        self.pick_Pose_Array = PoseArray()
        self.pick_Pose_Array.poses = [Pose() for i in range(8)]
        

       
        


        self.objectColorAndID =  {'karaage' :   ([1.0,0.0,0.0,1.0],0),   #Karaage marker's color
                             'onigiri' :    ([0.0,1.0,0.0,1.0],1),   #Onigiri marker's color
                             'box' :    ([0.0,0.0,1.0,1.0],2),   #Box marker's color
                             'bologna' :   ( [0.5,0.5,0.5,1.0],3),   #Chip marker's color  
                             'spaghetti' :   ( [0.04,0.68,0.5,1.0],4), #Bologna
                             'area1' :   ( [1.0,1.0,0.0,1.0],5),   #spaghetti
                             'area2' :   ( [1.1,1.1,1.1,1.0],6),   #udon
                             'area3' :   ( [0.0,0.0,1.0,1.0],7)}   #somen

        
    
    def nothing(x):
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
        self.model = msg.distortion_model
    def colorCallback(self, msg):
        self.color_image = ros_numpy.numpify(msg)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)

    def depthCallback(self, msg):
        self.depth_image = ros_numpy.numpify(msg).astype(np.uint16)
    def subPointCloudCallback(self,ros_point_cloud):
        self.fullPointCloud = ros_point_cloud
    def snapCallback(self,msg):
        print(msg.data)
        if not msg.data: #False
            self.sameSnap = 0
        self.snapMessage = msg.data







    def rgbToFloat(self, rgb):
        return struct.unpack('f', struct.pack('i', rgb))[0]

    def pcCallback(self, msg):
        self.fields = msg.fields

    def depthToPoints(self, depth_image, U, V):      
        x = (U - self.K[2])/self.K[0]
        y = (V - self.K[5])/self.K[4]      
        
        z = depth_image[V,U]*self.depth_scale
        x *= z
        y *= z
        # print(U,V,x,y,z)
        point = [x,y,z]
        return point

    def centroid(self,img):
        M   = cv2.moments(img)
        if  M["m00"] == 0 or M["m00"] == 0 :
            return
        
        cX  = int(M["m10"] / M["m00"])
        cY  = int(M["m01"] / M["m00"])
        return cX, cY
    
    def boundingBox(self,img): # ========================================================================= <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HERE
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #
        x, y, w, h = cv2.boundingRect(img)
        #hull = cv2.convexHull(img[0], returnPoints=False)
        # rect = cv2.minAreaRect(c)
        # box = cv2.boxPoints(rect)
        # box = np.int0(box)
        # ellipse = cv2.fitEllipse(img)
        return  ((x,y),(x+w,y+h))

    def takeDist(self,elem):
        return elem[1]
    
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

    def depthToPointsDistance(self, depth_image, U, V):
        V =  np.clip(V,0,self.height-1)
        U =  np.clip(U,0,self.width-1)  
        
        x = (U - self.K[2])/self.K[0]
        y = (V - self.K[5])/self.K[4]      
        U,V = self.find_nearest_nonzero(depth_image,(U,V))
        #print(vv,uu)
        z = depth_image[V,U]*self.depth_scale
        x *= z
        y *= z
        # print(U,V,x,y,z)
        point = [x,y,z]
        return point

    def depthToPoints(self, depth_image,color_image):
        [height, width] = depth_image.shape
        #print(depth_image.shape,color_image.shape)
        
        nx = np.linspace(0, width-1, width)
        ny = np.linspace(0, height-1, height)
        u, v = np.meshgrid(nx, ny)
        #print(u)
        x = (u.flatten() - self.K[2])/self.K[0]
        y = (v.flatten() - self.K[5])/self.K[4]

        z = depth_image.flatten() / 1000.0
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
        r = r[np.nonzero(z)]
        g = g[np.nonzero(z)]
        b = b[np.nonzero(z)]
        #rgb_int = rgb_int[np.nonzero(z)]
        #rgb_int = [self.rgb2hex(x,y,z) for x in r for y in g for z in b]
        #print(len(x),len(y),len(z),len(r),len(g),len(b),len(rgb_int))
        points = np.stack((x, y, z,rgb_int), axis = -1)
        #print(points)
        return points
    

    
    def publishSpaghettiPointCloud(self, pc_pub, points):
        #points_color = self.addColorToPoints(points, color)
        #filename = ('./cloudycloudu.ply')
        pc = point_cloud2.create_cloud(self.header, self.fields, points)
        pc_pub.publish(pc)
        self.spaghettiPointCloud = pc

    def publishPoseArray(self,pickArray):
        self.pose_array_pub.publish(pickArray)
    
    def clearPose(self):
        self.objectPose = {
            'karaage': Pose() ,'onigiri':Pose()  ,'box':Pose()   ,'bologna':Pose() ,'spaghetti': Pose() ,'area1': Pose() ,'area2': Pose()  ,'area3': Pose()
        }
        self.pick_Pose_Array.poses = [Pose() for i in range(8)]
        self.pick_Pose_Array.header.frame_id = "/camera_color_optical_frame"
        self.pick_Pose_Array.header.stamp = rospy.Time.now()
    
    
    
    
    
    def getResult(self):
        #sprint('getresult')
        self.clearPose()
        color_image, depth_image    = self.color_image, np.asanyarray(self.depth_image)
        
        
        depthForCloud = depth_image
        # ================================================================================ CHANGED ============
        # converted to 16bit image
        depth_image = depth_image.astype(np.uint16)
        #ipdb.set_trace()
        # ================================================================================ CHANGED ============

        img = color_image  
        cv2.imshow('src_rgb', color_image)
        # to make it more visually
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha=0.03),cv2.COLORMAP_JET)
        # image will be converted to 1 channel if this option is used
        # depth_colormap = depth_colormap.astype(np.uint16)
        depth_scale = np.hstack((color_image,depth_colormap))
        
        # ipdb.set_trace()
        
        ###for collect data only ###################
        ##########save file name and number###########
       
        #print(gram,old_num)
        ##########save file name and number###########
        
        ###############################################
        # print(depth_image)
        cv2.imshow('depth', depth_scale)
        result = inference_detector(self.modelDet, color_image)
        
        assert isinstance(self.class_names, (tuple, list))
        
        # ipdb.set_trace()
        # cv2.waitKey(1)
        wait_time=1
        show=True
        score_thr= r = cv2.getTrackbarPos('score','score_bar') / 100.0 #self.SCORE_THR
        # self.SCORE_THR = 0.65 #  cv2.getTrackbarPos('score','image')
        out_file=None
        
        if self.modelDet.with_mask:
            ms_bbox_result, ms_segm_result = result
            if isinstance(ms_bbox_result, dict):
                result = (ms_bbox_result['ensemble'],
                          ms_segm_result['ensemble'])
        else:
            if isinstance(result, dict):
                result = result['ensemble']
        

        img = mmcv.imread(img)
        img = img.copy()
        if isinstance(result, tuple):
            bbox_result, segm_result = result
            if isinstance(segm_result, tuple):
                segm_result = segm_result[0]  # ms rcnn
        else:
            bbox_result, segm_result = result, None
        bboxes = np.vstack(bbox_result)
        labels = [
            np.full(bbox.shape[0], i, dtype=np.int32)
            for i, bbox in enumerate(bbox_result)
        ]
        labels = np.concatenate(labels)
        # ipdb.set_trace()
        # draw segmentation masks
        #print(depth_image.shape,color_image.shape)
        if segm_result is not None and len(labels) > 0:  # non empty
            #print(depth_image.shape,color_image.shape)
            segms = mmcv.concat_list(segm_result)
            inds = np.where(bboxes[:, -1] > score_thr)[0]
            np.random.seed(42)

            color_masks = [
                np.random.randint(0, 256, (1, 3), dtype=np.uint8)
                for _ in range(max(labels) + 1)
            ]            
            for i in inds: #for each masked object
                i = int(i)
                object_name = self.objectDictID[labels[i]]
                #print(object_name)
                
                color_mask = color_masks[labels[i]]
                mask = segms[i]
                ##########forpointcloudsegmentation################################
                
                ####################################################################
                
                # for centroid calculation modified
                mask_int = mask.astype(np.uint8)*255
                # ipdb.set_trace()

                if self.centroid(mask_int) is None :
                    break
                cX, cY = self.centroid(mask_int)
                

                
                
                depth_point = self.depthToPointsDistance(depth_image=depth_image,U=cX,V=cY)
                if depth_point == [0.0,0.0,0.0] :
                    # print ('depth_point ', depth_point)
                    for j in range (5) :
                        depth_point = self.depthToPointsDistance(depth_image=depth_image,U=cX+np.random.randint (-5,5,(1,1)),
                            V=cY+np.random.randint (-5,5,(1,1)))
                        if depth_point != [0.0,0.0,0.0] :
                            break 
                if depth_point == [0.0,0.0,0.0] :
                    continue
                #distance = math.sqrt(math.pow(depth_point[0],2)+math.pow(depth_point[1],2)+math.pow(depth_point[2],2))     
                
                color_mask = color_masks[labels[i]]
                cv2.circle(img, (cX, cY), 5, (0, 0, 0), -1)

                # if labels[i] >= len(color_masks) :
                #     break
                #end centroid calculation
                
                img[mask] = img[mask] * 0.6 + color_mask * 0.4

                #save dataset i == 4
                # ================================================================================ CHANGED ============
                

                # ================================================================================ CHANGED ============

                ##--- Marker
                new_pose = Pose()
                new_pose.position.x     = depth_point[0]
                new_pose.position.y     = depth_point[1]
                new_pose.position.z     = depth_point[2]
                new_pose.orientation.x  = 0.0
                new_pose.orientation.y  = 0.0
                new_pose.orientation.z  = 0.0
                new_pose.orientation.w  = 1


                
                
                #############herer########################3
                ##--- END : Marker
                ##--- Append pos of each labels
                self.objectPose[object_name] = new_pose            
                ##--- END
            for objectName in self.objectPose.keys():
                send_traj_point_marker(marker_pub=self.object_pub[objectName], pose=self.objectPose[objectName],id= self.objectColorAndID[objectName][1], rgba_tuple=self.objectColorAndID[objectName][0])
            for key in self.objectPose:
                print(self.pick_Pose_Array.poses)
                self.pick_Pose_Array.poses[self.getFoodID[key]] = self.objectPose[key]
            self.publishPoseArray(self.pick_Pose_Array)
            
            
        
            


        # if out_file specified, do not show image in window
        if out_file is not None:
            show = False
        # draw bounding boxes
        
        bbox_color='green'
        text_color='green'
        thickness=1
        font_scale=0.5
        win_name='Hybrid Task Cascade'
        show=True
        wait_time=1

        mmcv.imshow_det_bboxes(
            img,
            bboxes,
            labels,
            class_names=self.modelDet.CLASSES,
            score_thr=score_thr,
            bbox_color=bbox_color,
            text_color=text_color,
            thickness=thickness,
            font_scale=font_scale,
            win_name=win_name,
            show=show,
            wait_time=wait_time,
            out_file=out_file)

        # ipdb.set_trace()
        # if not (show or out_file):
        #     return img
        
        # ipdb.set_trace()
        
        

    def getResult_ori(self):
        color_image, depth_image    = self.color_image, np.asanyarray(self.depth_image)
        img                         = color_image  
        cv2.imshow('src_rgb', color_image)
        result = inference_detector(self.modelDet, color_image)
        # return
        assert isinstance(self.class_names, (tuple, list))
        if isinstance(result, tuple):
            bbox_result, segm_result = result
        else:
            bbox_result, segm_result = result, None
        bboxes = np.vstack(bbox_result)  
        
        if segm_result is not None:
            labels = [
                np.full(bbox.shape[0], i, dtype=np.int32)
                for i, bbox in enumerate(bbox_result)
            ]
            labels = np.concatenate(labels)            
            segms = mmcv.concat_list(segm_result)
            inds = np.where(bboxes[:, -1] > self.SCORE_THR)[0]            
            dist_list = [[],[],[],[],[],[],[],[]]
            for i in inds:
                i = int(i)     
                # color_masks = [
                #     np.random.randint(0, 256, (1, 3), dtype=np.uint8)
                #     for _ in range(max(labels) + 1)
                # ]           
                color_masks = [np.array([[ 52, 160, 246]], dtype=np.uint8), np.array([[239,  53, 190]], dtype=np.uint8), np.array([[111,  60, 212]], dtype=np.uint8),np.array([[ 112, 153, 230]], dtype=np.uint8),np.array([[ 11, 174, 127]], dtype=np.uint8)]

                # ipdb.set_trace()
                self.mask = maskUtils.decode(segms[i]).astype(np.bool)                
                self.mask_int = self.mask.astype(np.uint8)*255
                # if labels[i] == 1 :
                #     cv2.imshow('bool_mask', self.mask*255.0)
                cX, cY = self.centroid(self.mask_int)

                depth_point = self.depthToPointsDistance(depth_image=depth_image,U=cX,V=cY)
                distance = math.sqrt(math.pow(depth_point[0],2)+math.pow(depth_point[1],2)+math.pow(depth_point[2],2))              
                

                color_mask = color_masks[labels[i]]
                cv2.circle(img, (cX, cY), 5, (0, 0, 0), -1)
                
                img[self.mask] = img[self.mask] * 0.5 + color_mask * 0.5
                
                ##--- Marker
                new_pose = Pose()
                new_pose.position.x     = depth_point[0]
                new_pose.position.y     = depth_point[1]
                new_pose.position.z     = depth_point[2]
                new_pose.orientation.x  = 0.0
                new_pose.orientation.y  = 0.0
                new_pose.orientation.z  = 0.0
                new_pose.orientation.w  = 1                
                ##--- END : Marker

                ##--- Append pos of each labels
                for index in range (8) :                    
                    if labels[i] == index: dist_list[index].append(dist(depth_point,distance))                 
                ##--- END
                
            rgba = [[1.0,0.0,0.0,1.0],   #Karaage marker's color
                    [0.0,1.0,0.0,1.0],   #Onigiri marker's color
                    [0.0,0.0,1.0,1.0],   #Box marker's color
                    [0.5,0.5,0.5,1.0],   #Chip marker's color  
                    [0.04,0.68,0.5,1.0], #Bologna
                    [0.0,0.0,0.0,0.0],   #spaghetti
                    [1.1,1.1,1.1,1.1],   #udon
                    [0.0,0.0,0.0,1.0],]   #somen   
            id = [1,1,1,1,1,1,1,1]

            for index in range (8) :
                # ipdb.set_trace()
                dist_list[index].sort(key=lambda x: (x.Z,x.distance))##--- Sort items in each category by Z then by 2D distance
                for DIST in dist_list[index]:
                    new_pose.position.x = DIST.X
                    new_pose.position.y = DIST.Y
                    new_pose.position.z = DIST.Z 
                    send_traj_point_marker(marker_pub=self.object_pub[index], pose=new_pose, id= id[index], rgba_tuple=rgba[index])
                    id[index] += 1  
            

        result_img = mmcv.imshow_det_bboxes(
            img,
            bboxes,
            labels,
            class_names=self.class_names,
            score_thr=0.7,
            show=True,
            wait_time=1)

        if result_img is not None:
        #     ipdb.set_trace()
            image_message = bridge.cv2_to_imgmsg(result_img, encoding="passthrough")   
            self.maskimg_pub.publish(image_message)


    def process(self):
        rospy.init_node('pointcloud_masking', anonymous=True)
        #r = rospy.Rate(60)
        rospy.Subscriber(self.CAMINFO['topic'], self.CAMINFO['msg'], self.camInfoCallback)
        rospy.Subscriber(self.COLOR['topic'], self.COLOR['msg'], self.colorCallback)
        rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['msg'], self.depthCallback)
        rospy.Subscriber(self.PC['topic'],      self.PC['msg'],         self.pcCallback)
        
        self.depth_scale    = 0.0010000000474974513

        self.object_pub = { 'karaage'      :  rospy.Publisher( '/object/karaage',    Marker , queue_size=10),
                                     'onigiri'      :  rospy.Publisher( '/object/onigiri',    Marker , queue_size=10),
                                     'box'          :  rospy.Publisher( '/object/box',        Marker , queue_size=10),
                                     'bologna'      :  rospy.Publisher( '/object/bologna',       Marker , queue_size=10),
                                     'spaghetti'    :  rospy.Publisher( '/object/spaghetti',    Marker , queue_size=10),
                                     'area1'        :  rospy.Publisher( '/object/area1',  Marker , queue_size=10),
                                     'area2'        :  rospy.Publisher( '/object/area2',       Marker , queue_size=10),
                                     'area3'        :  rospy.Publisher( '/object/area3',      Marker , queue_size=10)}
        self.maskimg_pub    =  rospy.Publisher( '/object/masking/boundary',    Image , queue_size=10)
        self.pose_array_pub = rospy.Publisher('/cam_pose_array', PoseArray, queue_size=1)

        while not rospy.is_shutdown():
            start = time.time()
            self.getResult()
            #r.sleep()
            end = time.time()
            seconds = end - start
            print(seconds)
            

if __name__ == '__main__':
    try:
        pointcloud_masking = PointcloudMasking()
        pointcloud_masking.process()

    except rospy.ROSInterruptException:
        pass
