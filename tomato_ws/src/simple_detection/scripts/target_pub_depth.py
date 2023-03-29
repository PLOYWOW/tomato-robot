#!/usr/bin/env python
import threading
import sys
import message_filters ### ADD THIS
import rospy
import numpy as np
import math
import glob
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import roslib; roslib.load_manifest('easy_markers')
from easy_markers.generator import *
import pyrealsense2 as rs
import argparse
import datetime
import sys
from geometry_msgs.msg import PointStamped,Pose
from utils.rs_process_depth import rs_process_depth
from utils import gui_creation
# import ipdb

###########1
# depth_w = int(rospy.get_param('/depth_width'))
depth_w = int(1280)
# depth_h = int(rospy.get_param('/depth_height'))
depth_h = int(720)

camera_resolution_depth = {'depth-w':depth_w,'depth-h':depth_h}

rs_image_depth = np.zeros((camera_resolution_depth['depth-w'],camera_resolution_depth['depth-h'],3), np.uint16)

ppx = ppy = fx = fy = 0
setting_isOK = False

###########2
if(camera_resolution_depth['depth-w'] == 1280 and camera_resolution_depth['depth-h'] == 720):
    setting_isOK = True
    ppx = 520.522
    ppy = 516.865
    fx  = 503.341
    fy  = 503.452
# if(camera_resolution_depth['depth-w'] == 640  and camera_resolution_depth['depth-h'] == 360):
#     setting_isOK = True
#     ppx = 321.3486328125
#     ppy = 179.70550537109375
#     fx  = 461.9239807128906
#     fy  = 462.3507080078125

if(not setting_isOK):
    print('*** Please set the correct instricsic')
    sys.exit()

###########3
################################# Have to set intrinsic
depth_scale = 0.0010000000474974513 #seems to be same in any resolution  //used      ############Use here
# depth_intrin = rs.intrinsics() # //used     ############Don't use here, used to deproject the depth on pixel coordinate
# depth_intrin.width = rs_image_rgb.shape[1]
# depth_intrin.height = rs_image_rgb.shape[0]
# depth_intrin.coeffs = [0, 0, 0, 0, 0]
# depth_intrin.ppx = ppx
# depth_intrin.ppy = ppy
# depth_intrin.fx = fx
# depth_intrin.fy = fy
# depth_intrin.model = rs.distortion.brown_conrady

#target detect args
enable_image_show = True
enable_fps_show   = True


def main():
    ###########???
    rospy.init_node('hand_tracking', anonymous=True)
    rospy.loginfo("--- Target Detection Start!")

    #######################################
    ##make new instance to subscribe image
    #######################################
    rsProcess_depth = rs_process_depth(camera_resolution_depth)
    print("NOW")
    # rsProcess_depth.activate()
    # cv2.imshow("depth_image from rs processes depth", rsProcess_depth.depth_image)
    # cv2.waitKey(50)

    print("-------------Exit from activate-------------")

    #############################################
    ##Create GUI for depth setting
    #############################################
    #define the initial depth param in CentiMeters
    # depth_MIN = np.array([47])
    # depth_MAX = np.array([100])
    #don't get the depth more than this
    depth_max_param = np.array([500])
    depth_param_name_list = ['DEPTH']
    #load the param
    rs_img_name = 'RS_SETTING'
    type_name = 'dist[cm]'
    if(len(glob.glob('./'+ rs_img_name+ 'dist*.npy')) == 2):
        depth_MIN = np.load('./' + rs_img_name + type_name + 'min.npy') #previous set depth dist
        depth_MAX = np.load('./' + rs_img_name + type_name + 'max.npy') #previous set depth dist -> -1
    else:
        depth_MIN = np.array([47])
        depth_MAX = np.array([100])
    
    print("********************")
    print("MIN = ",end="")
    print(depth_MIN)
    print("MAX = ",end="")
    print(depth_MAX)
    print("********************")
    trackbar_depth = gui_creation.GUI(rs_img_name,type_name,depth_param_name_list,depth_max_param,depth_MIN,depth_MAX)
                                    #img_name,type_name,param_name_list,max_param,MIN,MAX
                                    #'RS_SETTING','dist[cm]',['DEPTH'],np.array([500]),np.load('./' + rs_img_name + type_name + 'min.npy'),np.load('./' + rs_img_name + type_name + 'max.npy')
    trackbar_depth.create_trackbar()

    # print("Exit trackbar")
    # print("Hereeeeeeee 1")

    #time for fps calculation
    start_time = datetime.datetime.now()
    num_frames = 0
    # print("Hereeeeeeee 2")

    while not rospy.is_shutdown():
        # if rsProcess_depth.isOK == True:
        #     print(rsProcess_depth.isOK)
        # print("Hereeeeeeee 3")
        #get rgb,depth frames for synchronized frames
        if not rsProcess_depth.isOK: #if true -> not exit (not Flase)
            continue
        # print("\n\nHereeeeeeee 4\n\n")
        rsProcess_depth.isOK = False
        # print("Here55555555555555555555555555555555555555555")

        #start processiog
        img_depth = rsProcess_depth.depth_image

        depth_map = cv2.applyColorMap(cv2.convertScaleAbs(rs_image_depth, alpha=0.03), cv2.COLORMAP_JET)

        # Remove background - Set pixels further than clipping_distance to grey
        black_color = 0
        depth_image_3d = np.dstack((img_depth,img_depth,img_depth)) #depth image is 1 channel, color is 3 channels

        clipping_distance_in_centimeters = trackbar_depth.get_param_as_tuple()
        # print("min_clipping_distance in cm: ",end="")
        # print(clipping_distance_in_centimeters[0][0])
        # print("max_clipping_distance in cm: ",end="")
        # print(clipping_distance_in_centimeters[1][0])


        min_clipping_distance_in_meters = float(clipping_distance_in_centimeters[0][0]) / 100.0  #1 meter
        max_clipping_distance_in_meters = float(clipping_distance_in_centimeters[1][0]) / 100.0  #1 meter

        min_clipping_distance = min_clipping_distance_in_meters / depth_scale
        max_clipping_distance = max_clipping_distance_in_meters / depth_scale

        # Calculate Frames per second (FPS)
        num_frames += 1
        elapsed_time = (datetime.datetime.now() -
                        start_time).total_seconds()
        fps = num_frames / elapsed_time

        #Display depth image
        # print("Hereeeeeeeeee in displaying depth image")
        cv2.imshow("Depth image",depth_image_3d)
        cv2.imshow("MAP JET Depth Image", depth_map)
        # print("SHOW")
        cv2.waitKey(50)
        

if __name__ == '__main__':
    main()
    print("Close Target Detection Publisher")
    