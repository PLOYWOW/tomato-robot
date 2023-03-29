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
# from simple_detection.scripts.utils.rs_process_ver2 import rs_process_color
from utils.rs_process_ver2 import rs_process
# from utils.rs_process import rs_process
from utils import gui_creation

depth_w = int(1280)
depth_h = int(720)
color_w = int(1280)
color_h = int(720)

camera_resolution = {   'rgb-w':color_w,   'rgb-h':color_h,
                       'depth-w':depth_w,'depth-h':depth_h }

rs_image_rgb = np.zeros((camera_resolution['rgb-w'],camera_resolution['rgb-h'],3), np.uint8)
rs_image_depth = np.zeros((camera_resolution['depth-w'],camera_resolution['depth-h'],3), np.uint16)

ppx = ppy = fx = fy = 0
setting_isOK = False

if(camera_resolution['depth-w'] == 1280 and camera_resolution['depth-h'] == 720):
    setting_isOK = True
    #depth not color from azure
    ppx = 520.522 #cx optical center x
    ppy = 516.865 #cy optical center y
    fx  = 503.341 #fx focal length x
    fy  = 503.452 #fy focal length y

if(not setting_isOK):
    print('*** Please set the correct instricsic')
    sys.exit()

## set intrinsic param

depth_scale = 0.0010000000474974513 #seems to be same in any resolution  //used      ############Use here
depth_intrin = rs.intrinsics() # //used     ############Don't use here, used to deproject the depth on pixel coordinate
depth_intrin.width = rs_image_rgb.shape[1]
depth_intrin.height = rs_image_rgb.shape[0]
depth_intrin.coeffs = [0, 0, 0, 0, 0]
depth_intrin.ppx = ppx
depth_intrin.ppy = ppy
depth_intrin.fx = fx
depth_intrin.fy = fy
depth_intrin.model = rs.distortion.brown_conrady

#target detect args
enable_image_show = True
enable_fps_show   = True

def get_colored_area(cv_image,Lmask_MIN,Lmask_MAX,Umask_MIN,Umask_MAX):
    cv2.imshow('cv_image',cv_image)
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    cv2.imshow('hsv_image',hsv_image)
    # for _ in range(3):
    # median_hsv_img = cv2.bilateralFilter(hsv_image, d=9,sigmaColor=50,sigmaSpace=7)

    # median_hsv_img = cv2.medianBlur(hsv_image, 11)
    # median_bgr_img = cv2.cvtColor(median_hsv_img, cv2.COLOR_HSV2BGR)
    median_hsv_img = hsv_image
    median_bgr_img = cv_image
    # cv2.imshow('median_bgr_img',median_bgr_img)

    mask_image1 = cv2.inRange(median_hsv_img, Lmask_MIN, Lmask_MAX)
    mask_image2 = cv2.inRange(median_hsv_img, Umask_MIN, Umask_MAX)
    ## Merge the mask and extract the red regions
    redmask = cv2.bitwise_or(mask_image1, mask_image2 )
    cv2.imshow('redmask',redmask)
    extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=redmask)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2,2))
    redmask = cv2.erode(redmask, kernel, iterations = 1)
    redmask = cv2.dilate(redmask, kernel, iterations = 1)
    median_filtered_extracted_imag = cv2.bitwise_and(median_bgr_img, median_bgr_img, mask=redmask)

    area = cv2.countNonZero(redmask)

    return (area, median_filtered_extracted_imag)

def main():
    rospy.init_node('hand_tracking', anonymous=True)
    rospy.loginfo("--- Target Detection Start!")

    #######################################
    ##make new instance to subscribe image
    #######################################
    rsProcess = rs_process(camera_resolution)

    #############################################
    ##Create GUI for hsv mask setting
    #############################################
    #define the initial HSV param for mask1&2
    Lmask_MIN = np.array([0,50,20]) 
    Lmask_MAX = np.array([0,255,255]) 
    Umask_MIN = np.array([109,160,104]) 
    Umask_MAX = np.array([161,255,206]) 
    #the limit of H-channel is 180, others are 255
    hsv_max_param = np.array([180,255,255])
    hsv_param_name_list = ['H','S','V']
    #load the param
    rs_img_name = 'RS_SETTING'
    type_name1 = 'mask1--'
    type_name2 = 'mask2--'
    if(len(glob.glob('./'+ rs_img_name + 'mask*.npy')) == 4):
        Lmask_MIN = np.load('./' + rs_img_name + type_name1 + 'min.npy')
        Lmask_MAX = np.load('./' + rs_img_name + type_name1 + 'max.npy')
        Umask_MIN = np.load('./' + rs_img_name + type_name2 + 'min.npy')
        Umask_MAX = np.load('./' + rs_img_name + type_name2 + 'max.npy')
    else:
        Lmask_MIN = np.array([0,101,144])
        Lmask_MAX = np.array([7,255,240])
        Umask_MIN = np.array([170,160,104])
        Umask_MAX = np.array([180,255,206])

    #make instance to create trackbar
    trackbar_mask1 = gui_creation.GUI(rs_img_name,type_name1,hsv_param_name_list,hsv_max_param,Lmask_MIN,Lmask_MAX)
    trackbar_mask2 = gui_creation.GUI(rs_img_name,type_name2,hsv_param_name_list,hsv_max_param,Umask_MIN,Umask_MAX)
    trackbar_mask1.create_trackbar()
    trackbar_mask2.create_trackbar()

    #############################################
    ##Create GUI for depth setting
    #############################################
    depth_max_param = np.array([500])
    depth_param_name_list = ['DEPTH']
    #load the param
    type_name = 'dist[cm]'
    if(len(glob.glob('./'+ rs_img_name+ 'dist*.npy')) == 2):
        depth_MIN = np.load('./' + rs_img_name + type_name + 'min.npy') #[0]
        depth_MAX = np.load('./' + rs_img_name + type_name + 'max.npy') #[0]
    else:
        depth_MIN = np.array([47])
        depth_MAX = np.array([100])
    trackbar_depth = gui_creation.GUI(rs_img_name,type_name,depth_param_name_list,depth_max_param,depth_MIN,depth_MAX)
    trackbar_depth.create_trackbar()

    #time for fps calculation
    start_time = datetime.datetime.now()
    num_frames = 0

    while not rospy.is_shutdown():
        #get rgb,depth frames for synchronized frames
        if not rsProcess.isOK: 
            continue

        rsProcess.isOK = False

        #start processiog
        img_bgr = rsProcess.bgr_image ####################Image #To applyfilter #To getcolor #To find contour
        img_rgb = rsProcess.rgb_image
        img_depth = rsProcess.depth_image

        ##Sometimes the depth.shape could be ((height,width,chnnel)) which are swapped(height<->width)
        if(not (img_bgr.shape[0] == img_depth.shape[0])):
            print('*******************************************************************')
            print('**** THE CAMERA INFO IS INVALID **** restart the camera launch file' )
            sys.exit()        

        # cv2.imshow("RGB image", img_rgb)
        cv2.imshow("BGR image", img_bgr)
        cv2.imshow("Depth image", img_depth)
        cv2.waitKey(50)

if __name__ == '__main__':
    main()
    print("Close Target Detection Publisher")

print("-------------OKAY-------------")