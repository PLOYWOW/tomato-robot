#!/usr/bin/env python
import numpy as np
import os
import math
import time
import cv2
import json
import mmcv
import struct
import timeit

from mmdet.models import build_detector
from mmdet.apis import inference_detector, init_detector

import pycocotools.mask as maskUtils
from pycocotools.coco import COCO as coco

import rospy
import ros_numpy
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import matplotlib.pyplot as plt

#import ipdb
#ipdb.set_trace()
class PointcloudMasking(object):
    def __init__(self):
        self.CAMINFO = {'topic': '/camera/color/camera_info', 'msg': CameraInfo}
        self.COLOR = {'topic': '/camera/color/image_raw', 'msg': Image}
        self.DEPTH = {'topic': '/camera/aligned_depth_to_color/image_raw', 'msg': Image}

        self.H = 720
        self.W = 1280
        self.K = CameraInfo().K
        self.header = Header()
        self.fields = [PointField('x', 0, 7, 1), PointField('y', 4, 7, 1), PointField('z', 8, 7, 1), PointField('rgb', 16, 7, 1)]
        self.points = []
        self.pc = PointCloud2()

        self.rgb_red = 0xff0000
        self.rgb_green = 0x00ff00
        self.rgb_blue = 0x0000ff
        self.rgb_yellow = 0xffff00
        self.rgb_cyan = 0x00ffff
        self.rgb_purple = 0xff00ff
        self.rgb_a = 0xffffff
        self.rgb_b = 0x0ff000
        self.rgb_c = 0x0fff00
        self.rgb_d = 0xffffff

        self.color_image = np.empty((self.H, self.W ,3), dtype=np.uint8)
        self.depth_image = np.empty((self.H, self.W), dtype=np.uint8)

        PATH_TO_PROJECT = '/home/yaskawa/worksp/camera_ws/src/realsense-ros/realsense2_camera/scripts/mmdetection/newbox_food'
        ANNOTATION_FILENAME = 'trainval.json'
        CHECKPOINT_FILENAME = 'epoch_14000.pth'
        CONFIG_FILENAME = 'tsuji_htc_newbox_food.py'

        annotation_file = os.path.join(PATH_TO_PROJECT, ANNOTATION_FILENAME)
        json_file = open(annotation_file)
        coco = json.load(json_file)
        checkpoint_file = os.path.join(PATH_TO_PROJECT, CHECKPOINT_FILENAME)
        config_file = os.path.join(PATH_TO_PROJECT, CONFIG_FILENAME)

        self.class_names = [category['name'] for category in coco['categories']]
        self.model = init_detector(config_file, checkpoint_file)
        self.SCORE_THR = 0.8

        self.pick_Pose = Pose()
        self.pick_Pose.position.x = 0.0
        self.pick_Pose.position.y = 0.0
        self.pick_Pose.position.z = 0.0
        self.null_pick_Pose_Array = PoseArray()
        for p in range(8):
            self.null_pick_Pose_Array.poses.append(self.pick_Pose)

    def camInfoCallback(self, msg):
        self.header = msg.header
        self.K = msg.K

    def colorCallback(self, msg):
        self.color_image = ros_numpy.numpify(msg)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)

    def depthCallback(self, msg):
        self.depth_image = ros_numpy.numpify(msg)

    def rgbToFloat(self, rgb):
        return struct.unpack('f', struct.pack('i', rgb))[0]

    def depthToPoints(self, depth_image):
        [height, width] = depth_image.shape
        nx = np.linspace(0, width-1, width)
        ny = np.linspace(0, height-1, height)
        u, v = np.meshgrid(nx, ny)
        x = (u.flatten() - self.K[2])/self.K[0]
        y = (v.flatten() - self.K[5])/self.K[4]

        z = depth_image.flatten() / 1000.0
        x = np.multiply(x,z)
        y = np.multiply(y,z)

        x = x[np.nonzero(z)]
        y = y[np.nonzero(z)]
        z = z[np.nonzero(z)]

        points = np.stack((x, y, z), axis = -1)

        return points

    def addColorToPoints(self, points, color):
        float_rgb = self.rgbToFloat(color)
        rgb = np.full((len(points),), float_rgb)
        points_color = np.c_[points, rgb]
        print(color)
        return points_color

    def centroid(self, points):
        x = [p[0] for p in points]
        y = [p[1] for p in points]
        z = [p[2] for p in points]

        cx = np.average(x)
        cy = np.average(y)
        cz = np.average(z)
        centroid = [cx, cy, cz]

        return centroid

    def createLabel(self, text, position, id = 0, duration = 5.0, color = [1.0, 1.0, 1.0]):
        marker = Marker()
        marker.header = self.header
        marker.id = id
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = text
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.lifetime = rospy.Duration(duration)
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = position[0] + 0.05
        marker.pose.position.y = position[1] + 0.05
        marker.pose.position.z = position[2] - 0.15

        return marker

    def publishPointcloud(self, pc_pub, points, color):
        points_color = self.addColorToPoints(points, color)
        pc = point_cloud2.create_cloud(self.header, self.fields, points_color)
        pc_pub.publish(pc)

    def publishMarker(self, marker_pub, marker):
        marker_pub.publish(marker)

    def publishPoseArray(self,pickArray):
        self.pose_array_pub.publish(pickArray)

    def process(self):
        color_image, depth_image    = self.color_image, np.asanyarray(self.depth_image)
        #Show image


        # ================================================================================ CHANGED ============
        # converted to 16bit image
        #depth_image = depth_image.astype(np.uint16)
        #ipdb.set_trace()
        # ================================================================================ CHANGED ============
        cv2.imshow('src_rgb', color_image)
        cv2.waitKey(10)
        







        result = inference_detector(self.model, color_image)
        assert isinstance(self.class_names, (tuple, list))
        if isinstance(result, tuple):
            bbox_result, segm_result = result
        else:
            bbox_result, segm_result = result, None
        bboxes = np.vstack(bbox_result)

        #pick_Pose_Array = self.null_pick_Pose_Array
        
        pick_Pose_Array = PoseArray()
        pick_Pose_Array.poses = self.null_pick_Pose_Array.poses.copy()
        
        pick_Pose_Array.header.frame_id = "/camera_color_optical_frame"
        pick_Pose_Array.header.stamp = rospy.Time.now()

        

        if segm_result is not None:
            print('detected!')
            labels = [
                np.full(bbox.shape[0], i, dtype=np.int32)
                for i, bbox in enumerate(bbox_result)
            ]
            labels = np.concatenate(labels)
            segms = mmcv.concat_list(segm_result)
            inds = np.where(bboxes[:, -1] > self.SCORE_THR)[0]
            for i in inds:
                pick_Pose = Pose()
                i = int(i)
                masked_depth = np.zeros((self.H, self.W), dtype=np.uint16)
                mask = segms[i] #maskUtils.decode(segms[i]).astype(np.bool)
                mask_int = mask.astype(np.uint16)*255
                ret, thresh = cv2.threshold(mask_int, 127, 255, 0)
                masked_depth[np.nonzero(thresh)] = depth_image[np.nonzero(thresh)]

                points = self.depthToPoints(masked_depth)
                self.header.stamp = rospy.Time.now()
                marker_label = self.createLabel(self.class_names[labels[i]], self.centroid(points))

                if labels[i] == 0: 
                    self.publishPointcloud(self.pc_pub_karaage, points, self.rgb_red)
                    self.marker_pub_karaage.publish(marker_label)
                    center_point = self.centroid(points)
                    pick_Pose.position.x = center_point[0]
                    pick_Pose.position.y = center_point[1]
                    pick_Pose.position.z = center_point[2]
                    pick_Pose_Array.poses[labels[i]] = pick_Pose
                    
                elif labels[i] == 1: 
                    self.publishPointcloud(self.pc_pub_onigiri, points, self.rgb_green)
                    self.marker_pub_onigiri.publish(marker_label)
                    center_point = self.centroid(points)
                    pick_Pose.position.x = center_point[0]
                    pick_Pose.position.y = center_point[1]
                    pick_Pose.position.z = center_point[2]
                    pick_Pose_Array.poses[labels[i]] = pick_Pose

                elif labels[i] == 2: 
                    self.publishPointcloud(self.pc_pub_BentoBox, points, self.rgb_red)
                    self.marker_pub_BentoBox.publish(marker_label)
                    center_point = self.centroid(points)
                    pick_Pose.position.x = center_point[0]
                    pick_Pose.position.y = center_point[1]
                    pick_Pose.position.z = center_point[2]
                    pick_Pose_Array.poses[labels[i]] = pick_Pose

                elif labels[i] == 3: 
                    self.publishPointcloud(self.pc_pub_bologna, points, self.rgb_yellow)
                    self.marker_pub_bologna.publish(marker_label)
                    center_point = self.centroid(points)
                    pick_Pose.position.x = center_point[0]
                    pick_Pose.position.y = center_point[1]
                    pick_Pose.position.z = center_point[2]
                    pick_Pose_Array.poses[labels[i]] = pick_Pose

                elif labels[i] == 4: 
                    self.publishPointcloud(self.pc_pub_spaghetti, points, self.rgb_cyan)
                    self.marker_pub_spaghetti.publish(marker_label)
                    center_point = self.centroid(points)
                    pick_Pose.position.x = center_point[0]
                    pick_Pose.position.y = center_point[1]
                    pick_Pose.position.z = center_point[2]
                    pick_Pose_Array.poses[labels[i]] = pick_Pose

                elif labels[i] == 5: #and i == 0: 
                    self.publishPointcloud(self.pc_pub_area1, points, self.rgb_purple)
                    self.marker_pub_area1.publish(marker_label)
                    center_point = self.centroid(points)
                    pick_Pose.position.x = center_point[0]
                    pick_Pose.position.y = center_point[1]
                    pick_Pose.position.z = center_point[2]
                    pick_Pose_Array.poses[labels[i]] = pick_Pose

                #elif labels[i] == 6 #and i == 1: 
                    # self.publishPointcloud(self.pc_pub_spaghetti1, points, self.a)
                    # self.marker_pub_spaghetti1.publish(marker_label)
                elif labels[i] == 6: 
                    self.publishPointcloud(self.pc_pub_area2, points, self.rgb_a)
                    self.marker_pub_area2.publish(marker_label)
                    center_point = self.centroid(points)
                    pick_Pose.position.x = center_point[0]
                    pick_Pose.position.y = center_point[1]
                    pick_Pose.position.z = center_point[2]
                    pick_Pose_Array.poses[labels[i]] = pick_Pose

                elif labels[i] == 7: 
                    self.publishPointcloud(self.pc_pub_area3, points, self.rgb_b)
                    self.marker_pub_area3.publish(marker_label)
                    center_point = self.centroid(points)
                    pick_Pose.position.x = center_point[0]
                    pick_Pose.position.y = center_point[1]
                    pick_Pose.position.z = center_point[2]
                    pick_Pose_Array.poses[labels[i]] = pick_Pose

                # elif labels[i] == 8: 
                #     self.publishPointcloud(self.pc_pub_yakisoba, points, self.rgb_c)
                #     self.marker_pub_yakisoba.publish(marker_label)
                # elif labels[i] == 9: 
                #     self.publishPointcloud(self.pc_pub_macaroni, points, self.rgb_d)
                #     self.marker_pub_macaroni.publish(marker_label)
                
        self.publishPoseArray(pick_Pose_Array)

    def main(self):
        rospy.init_node('pointcloud_masking', anonymous=True)

        rospy.Subscriber(self.CAMINFO['topic'], self.CAMINFO['msg'], self.camInfoCallback)
        rospy.Subscriber(self.COLOR['topic'], self.COLOR['msg'], self.colorCallback)
        rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['msg'], self.depthCallback)

        self.pc_pub_karaage = rospy.Publisher('/pointcloud_karaage', PointCloud2, queue_size=1)
        self.pc_pub_onigiri = rospy.Publisher('/pointcloud_onigiri', PointCloud2, queue_size=1)
        self.pc_pub_BentoBox = rospy.Publisher('/pointcloud_BentoBox', PointCloud2, queue_size=1)
        self.pc_pub_bologna = rospy.Publisher('/pointcloud_bologna', PointCloud2, queue_size=1)
        self.pc_pub_spaghetti = rospy.Publisher('/pointcloud_spaghetti', PointCloud2, queue_size=1)
        self.pc_pub_area1 = rospy.Publisher('/pointcloud_area1', PointCloud2, queue_size=1)
        self.pc_pub_area2 = rospy.Publisher('/pointcloud_area2', PointCloud2, queue_size=1)
        self.pc_pub_area3 = rospy.Publisher('/pointcloud_area3', PointCloud2, queue_size=1)

        self.marker_pub_karaage = rospy.Publisher('/label_karaage', Marker, queue_size=1)
        self.marker_pub_onigiri = rospy.Publisher('/label_onigiri', Marker, queue_size=1)
        self.marker_pub_BentoBox = rospy.Publisher('/label_BentoBox', Marker, queue_size=1)
        self.marker_pub_bologna = rospy.Publisher('/label_bologna', Marker, queue_size=1)
        self.marker_pub_spaghetti = rospy.Publisher('/label_spaghetti', Marker, queue_size=1)
        self.marker_pub_area1 = rospy.Publisher('/label_area1', Marker, queue_size=1) ##
        self.marker_pub_area2 = rospy.Publisher('/label_area2', Marker, queue_size=1)
        self.marker_pub_area3 = rospy.Publisher('/label_area3', Marker, queue_size=1)

        self.pose_array_pub = rospy.Publisher('/cam_pose_array', PoseArray, queue_size=1)

        
        while not rospy.is_shutdown():
            self.process()
            

if __name__ == '__main__':
    try:
        pointcloud_masking = PointcloudMasking()
        pointcloud_masking.main()
    
    except rospy.ROSInterruptException: pass