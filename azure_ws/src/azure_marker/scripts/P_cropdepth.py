#!/usr/bin/env python


import numpy as np
import os
import math
import time
import cv2

import struct
from cv_bridge import CvBridge
bridge = CvBridge()

import rospy
import ros_numpy
from std_msgs.msg import Header,String
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Image
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker,MarkerArray

import glob
from geometry_msgs.msg import Point,Pose,PoseStamped,PoseArray
import ctypes

import open3d as o3d

import matplotlib.pyplot as plt

from numpy.lib.stride_tricks import as_strided


class DepthImageHandler(object):
    def __init__(self):
        self.CAMINFO = {'topic': '/depth_to_rgb/camera_info', 'msg': CameraInfo}
        self.isCamInfo = False

        self.COLOR = {'topic': '/rgb/image_raw', 'msg': Image}
        self.DEPTH = {'topic': '/depth_to_rgb/image_raw', 'msg': Image}
        self.PC = {'topic': '/camera/depth/color/points', 'msg': PointCloud2}
        
        self.H = 720
        self.W = 1280
        self.cropSize = 2
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
        self.cropPointCloud = o3d.geometry.PointCloud() 

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
        self.isCamInfo = True
    def colorCallback(self, msg):
        self.color_image = ros_numpy.numpify(msg)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)

    def depthCallback(self, msg):
        self.depth_image = ros_numpy.numpify(msg).astype(np.uint16)
    

    def rgbToFloat(self, rgb):
        return struct.unpack('f', struct.pack('i', rgb))[0]

    def pcCallback(self, msg):
        self.fields = msg.fields



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
        x = (u.flatten() - self.K[2]/self.cropSize)/self.K[0]
        y = (v.flatten() - self.K[5]/self.cropSize)/self.K[4]

        z = depth_image.flatten() / 1000.0
        color_points = np.reshape(color_image, [-1,3])
        ##for only non-zero pointcloud
        #color_points = color_points[np.nonzero(z)]
        
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
        

        ##for only non-zero pointcloud
        # x = x[np.nonzero(z)]
        # y = y[np.nonzero(z)]
        # z = z[np.nonzero(z)]
        # r = r[np.nonzero(z)]
        # g = g[np.nonzero(z)]
        # b = b[np.nonzero(z)]

    
        #rgb_int = rgb_int[np.nonzero(z)]
        #rgb_int = [self.rgb2hex(x,y,z) for x in r for y in g for z in b]
        print(len(x),len(y),len(z),len(r),len(g),len(b),len(rgb_int))
        points = np.stack((x, y, z,rgb_int), axis = -1)
        print(points.shape)
        return points
    

    def publishCropPointCloud(self, pc_pub, points):
        pc = point_cloud2.create_cloud(self.header, self.fields, points)
        self.cropPointCloud = pc
        pc_pub.publish(pc)

    def center_crop(self,img, dim):
        width, height = img.shape[1], img.shape[0]
        # process crop width and height for max available dimension
        crop_width = dim[0] if dim[0]<img.shape[1] else img.shape[1]
        crop_height = dim[1] if dim[1]<img.shape[0] else img.shape[0] 
        mid_x, mid_y = int(width/2), int(height/2)
        cw2, ch2 = int(crop_width/2), int(crop_height/2) 
        crop_img = img[mid_y-ch2:mid_y+ch2, mid_x-cw2:mid_x+cw2]
        return crop_img
    def sliding_window(self,img, window_size):
        """ Construct a sliding window view of the array"""
        arr = np.asarray(img)
        window_size = int(window_size)
        if arr.ndim != 2:
            raise ValueError("need 2-D input")
        if not (window_size > 0):
            raise ValueError("need a positive window size")
        shape = (arr.shape[0] - window_size + 1,
                arr.shape[1] - window_size + 1,
                window_size, window_size)
        if shape[0] <= 0:
            shape = (1, shape[1], arr.shape[0], shape[3])
        if shape[1] <= 0:
            shape = (shape[0], 1, shape[2], arr.shape[1])
        strides = (arr.shape[1]*arr.itemsize, arr.itemsize,
                arr.shape[1]*arr.itemsize, arr.itemsize)
        return as_strided(arr, shape=shape, strides=strides)

    def cell_neighbors(self,depth_image, i, j):
        width, height = depth_image.shape[1], depth_image.shape[0]
        widthBound = width-1
        heightBound = height-1
        filterBlock = np.zeros([5, 5, 1], np.uint16)
        for x in range(-2,3,1):
            for y in range(-2,3,1):
                xSearch = i + x
                ySearch = j + y
                if (xSearch > 0 and xSearch < widthBound and ySearch > 0 and ySearch < heightBound):
                    filterBlock[y+2,x+2] = depth_image[ySearch,xSearch]
        return filterBlock

    def fillDepthImage(self,depth_image):
        width, height = depth_image.shape[1], depth_image.shape[0]
        widthBound = width-1
        heightBound = height-1
        filledDepth = depth_image
        filterBlock = np.zeros([5, 5, 1], np.uint16)

        meanFilter = 0
        numZeroPixel = 0

        for u in range(width):
            for v in range(height):
                if math.isclose(filledDepth[v,u],0.0,rel_tol=1e-6):
                    numZeroPixel += 1

                    filterBlock = self.cell_neighbors(filledDepth,u,v)
                    filterBlock = filterBlock[np.nonzero(filterBlock)] #find non zero in filter
                    if len(filterBlock) == 0:
                        meanFilter = 0
                    else:
                        meanFilter = np.mean(filterBlock)
                    filledDepth[v,u] = meanFilter

                    # print(filterBlock.shape)
                    # print(scipy.stats.mode(filterBlock))
                    # plt.imshow(filterBlock)
                    # plt.show()
                    
        return filledDepth,numZeroPixel
    
    

        
    
    def GetCropImage(self):

        color_image, depth_image    = self.color_image, np.asanyarray(self.depth_image)

        depthForCloud = depth_image
        # ================================================================================ CHANGED ============
        # converted to 16bit image
        depth_image = depth_image.astype(np.uint16)
        # plt.imshow(depth_image)
        # plt.show()

        cropDepthImage = self.center_crop(depth_image,[self.W//self.cropSize,self.H//self.cropSize])
        filledCropDepthImage,numZeroPixel = self.fillDepthImage(cropDepthImage)
        print(f"num zero:{numZeroPixel}")

        cropColorImage = self.center_crop(color_image,[self.W//self.cropSize,self.H//self.cropSize])
        


        if self.isCamInfo:
            self.cropPointCloud = self.depthToPoints(filledCropDepthImage,cropColorImage)
            self.publishCropPointCloud(self.cropPointCloudPublisher,self.cropPointCloud)
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
        cv2.waitKey(60)
        
    def process(self):
        rospy.init_node('pointcloud_masking', anonymous=True)
        #r = rospy.Rate(60)
        rospy.Subscriber(self.CAMINFO['topic'], self.CAMINFO['msg'], self.camInfoCallback)
        rospy.Subscriber(self.COLOR['topic'], self.COLOR['msg'], self.colorCallback)
        rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['msg'], self.depthCallback)
        rospy.Subscriber(self.PC['topic'],      self.PC['msg'],         self.pcCallback)
        
        self.depth_scale    = 0.0010000000474974513

        ###publisher
        self.cropPointCloudPublisher = rospy.Publisher('/croppedDepthImage', PointCloud2, queue_size=1)


        while not rospy.is_shutdown():
                self.GetCropImage()
         
if __name__ == '__main__':
    try:
        _depthImageHandler = DepthImageHandler()
        _depthImageHandler.process()

    except rospy.ROSInterruptException:
        pass
