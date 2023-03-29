#!/usr/bin/env python

import numpy as np
import os
import math
import time
import cv2
#import json
#import mmcv
import struct
from cv_bridge import CvBridge
bridge = CvBridge()
#from mmdet.apis import inference_detector, init_detector

#import pycocotools.mask as maskUtils
#from pycocotools.coco import COCO as coco

import rospy
import ros_numpy
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Image
from visualization_msgs.msg import Marker,MarkerArray


import glob
#import pyrealsense2 as rs
#import torch
from geometry_msgs.msg import Point,Pose
#import ipdb
#import tifffile as tiff
#import open3d as o3d
#import ctypes

import sensor_msgs.point_cloud2 as pc2
import pcl
# import tf2_ros
# import tf2_py as tf2
# from sensor_msgs.msg import PointCloud2
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

# tf_buffer = tf2_ros.Buffer()
# tf_listener = tf2_ros.TransformListener(tf_buffer)


class PointCloudTransformer(object):
    def __init__(self):
        self.CAMINFO = {'topic': '/camera/color/camera_info', 'msg': CameraInfo}
        self.COLOR = {'topic': '/camera/color/image_raw', 'msg': Image}
        self.DEPTH = {'topic': '/camera/aligned_depth_to_color/image_raw', 'msg': Image}
        self.PC = {'topic': '/camera/depth/color/points', 'msg': PointCloud2}
        ###publisher&subscriber###
        self.pc_pub_area3 = rospy.Publisher('/pointcloud_newworld', PointCloud2, queue_size=1)
        self.cloud_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2,self.subPointCloudCallback,queue_size=1, buff_size=52428800)
        ###############
        self.pc = PointCloud2()

    def subPointCloudCallback(self,ros_point_cloud):
        self.pc = ros_point_cloud
    
    def getTranformed(self):
        self.pc_pub_area3.publish(self.pc)



    def process(self):
        rospy.init_node('pointcloud_transforming', anonymous=True)
        
        self.depth_scale    = 0.0010000000474974513
        self.object_pub     = [rospy.Publisher( '/object/karaage',    Marker , queue_size=10),
                               rospy.Publisher( '/object/onigiri',    Marker , queue_size=10),
                               rospy.Publisher( '/object/box',        Marker , queue_size=10),
                               rospy.Publisher( '/object/bologna',       Marker , queue_size=10),
                               rospy.Publisher( '/object/spaghetti',    Marker , queue_size=10),
                               rospy.Publisher( '/object/area1',  Marker , queue_size=10),
                               rospy.Publisher( '/object/area2',       Marker , queue_size=10),
                               rospy.Publisher( '/object/area3',      Marker , queue_size=10)]
        self.maskimg_pub    =  rospy.Publisher( '/object/masking',    Image , queue_size=10)
        
        while not rospy.is_shutdown():
            self.getTranformed()
            print('Transformer is tranforming a transformed point')

    
        
        

if __name__ == '__main__':
    try:
        pointcloud_masking = PointCloudTransformer()
        pointcloud_masking.process()

    except rospy.ROSInterruptException:
        pass



    # try:
    #     trans = tf_buffer.lookup_transform(target_frame, msg.header.frame_id,
    #                                        msg.header.stamp,
    #                                        rospy.Duration(timeout))
    # except tf2.LookupException as ex:
    #     rospy.logwarn(ex)
    #     return
    # except tf2.ExtrapolationException as ex:
    #     rospy.logwarn(ex)
    #     return
    # cloud_out = do_transform_cloud(msg, trans)