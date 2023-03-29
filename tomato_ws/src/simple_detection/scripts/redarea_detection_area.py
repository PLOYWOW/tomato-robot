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
from utils.rs_process_ver3 import rs_process
# from utils.rs_process import rs_process
from utils import gui_creation
from utils.segment import segment
from std_msgs.msg import Bool, Int32

threshold_redarea = 90 #in percent %

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
depth_scale = 0.0010000000474974513 #seems to be same in any resolution  //used  
depth_intrin = rs.intrinsics() 
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
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    median_hsv_img = hsv_image
    median_bgr_img = cv_image

    Lmask_MIN = np.asarray(Lmask_MIN)
    Lmask_MAX = np.asarray(Lmask_MAX)
    Umask_MIN = np.asarray(Umask_MIN)
    Umask_MAX = np.asarray(Umask_MAX)
    
    mask_image1 = cv2.inRange(median_hsv_img, Lmask_MIN, Lmask_MAX)
    mask_image2 = cv2.inRange(median_hsv_img, Umask_MIN, Umask_MAX)

    ## Merge the mask and extract the red regions
    redmask = cv2.bitwise_or(mask_image1, mask_image2 )
    # cv2.imshow('redmask',redmask)
    extracted_image = cv2.bitwise_and(cv_image, cv_image, mask=redmask)
    # cv2.imshow('extracted_image',extracted_image)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2,2))
    redmask = cv2.erode(redmask, kernel, iterations = 1)
    redmask = cv2.dilate(redmask, kernel, iterations = 1)
    median_filtered_extracted_imag = cv2.bitwise_and(median_bgr_img, median_bgr_img, mask=redmask)

    kernel = np.ones((5,5),np.uint8)
    median_filtered_extracted_imag = cv2.erode(median_filtered_extracted_imag,kernel,iterations=3)
    kernel = np.ones((5,5),np.uint8)
    median_filtered_extracted_imag = cv2.erode(median_filtered_extracted_imag,kernel,iterations=1)

    # cv2.imshow('red detected image',median_filtered_extracted_imag)

    area = cv2.countNonZero(redmask) #Number of non-zero elements

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

    #############################################
    #Marker Publisher Initialization
    #############################################
    # pub=rospy.Publisher('/target_marker', Marker,queue_size=1)
    pub=rospy.Publisher('/target_tomato', Marker,queue_size=1)
    redArea_pub = rospy.Publisher("red_area",Int32,queue_size=1)
    flag_area = Bool()
    hand_mark = MarkerGenerator()
    hand_mark.type = Marker.SPHERE_LIST
    hand_mark.scale = [.03]*3
    hand_mark.frame_id = '/camera_color_optical_frame'

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

        cv2.namedWindow("3",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("3",(int(1280*0.85),int(360*0.85)))
        cv2.moveWindow("3",500,700)
        cv2.putText(img_bgr, "input image", (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (77, 255, 9), 2)
        cv2.imshow("3",img_bgr)
        
        depth_map = cv2.applyColorMap(cv2.convertScaleAbs(rs_image_depth, alpha=0.03), cv2.COLORMAP_JET)

        #Remove background - Set pixels further than clipping_distance to grey
        black_color = 0
        depth_image_3d = np.dstack((img_depth,img_depth,img_depth)) #depth image is 1 channel, color is 3 channels

        #apply filter to extract the red color
        Lmask_MIN, Lmask_MAX = trackbar_mask1.get_param_as_tuple()
        Umask_MIN, Umask_MAX = trackbar_mask2.get_param_as_tuple()

        if(True):       
            # left_tomato_cropped_img = bg_removed[:,0:color_w/2]
            # left_tomato_cropped_img = img_bgr[:,0:color_w/2]
            left_tomato_cropped_img = img_bgr

            #get the result val and image obj
            red_area, extracted_image = get_colored_area(left_tomato_cropped_img,Lmask_MIN,Lmask_MAX,Umask_MIN,Umask_MAX)
            #red_area = no. of non-zero elements
            #extracted_image = red extracted image

            total_pixel = 1280*720



            red_area_perc = (red_area/total_pixel)*100
            
            if red_area_perc >= threshold_redarea:
                flag_area.data = True
            else:
                flag_area.data = False

            print("red_area_percent: ",end="")
            print(red_area_perc)
            print("flag_area: ",end="")
            print(flag_area.data)

            # redArea_pub.publish(flag_area)
            redArea_pub.publish(red_area)

            #convert red-area to gray scale
            gray_extracted_image = cv2.cvtColor(extracted_image, cv2.COLOR_RGB2GRAY) #w/o segment
        
            #convert red-area-gray scale to binary img
            ret, threshold_img = cv2.threshold(gray_extracted_image, 1, 255, cv2.THRESH_BINARY)

            if(ret):
                # find contours
                contours, hierarchy = cv2.findContours(threshold_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                
                # get size of contour
                contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
                
                if(len(contours)>0):
                    #select biggest one
                    biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
                    biggest_contour_index = contour_sizes.index(max(contour_sizes, key=lambda x: x[0]))
                    cv2.drawContours(extracted_image, contours, -1, (255, 0, 0), 2)
                    (x,y),radius = cv2.minEnclosingCircle(biggest_contour)
                    #Get the total pixel of biggest_contour
                    red_area = cv2.contourArea(biggest_contour)
                    center = (int(x),int(y))
                    # print("center of biggest contour: ",end="")
                    # print(center)
                    radius = int(radius)
                    num_of_cnts = len(contours)
                    cv2.circle(extracted_image, (int(x),int(y)),10,(0,255,0),-1)

                    #display window
                    if (enable_image_show):
                        cv2.putText(extracted_image, 'red pixel(total): '+str(red_area), (10,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
                        cv2.putText(extracted_image, 'radius: ' + str(radius), (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
                        cv2.putText(extracted_image, 'num of cnts: ' + str(num_of_cnts), (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)

        # Calculate Frames per second (FPS)
        num_frames += 1
        elapsed_time = (datetime.datetime.now() -
                        start_time).total_seconds()
        fps = num_frames / elapsed_time

        #display window
        if (enable_image_show):
            # Display FPS on frame
            if (enable_fps_show):
                # cv2.putText(bg_removed, str(fps), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
                cv2.putText(extracted_image, "fps: "+str(fps), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
        else:
            if (enable_fps_show):
                print("frames processed: ",  num_frames, "elapsed time: ", elapsed_time, "fps: ", str(int(fps)))

        cv2.namedWindow("1",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("1",(int(640*1),int(360*1)))
        cv2.moveWindow("1",500,0)
        cv2.putText(extracted_image, "with marker", (40, 700), cv2.FONT_HERSHEY_SIMPLEX, 2, (77, 255, 9), 2)
        cv2.imshow("1",extracted_image)
        
        cv2.namedWindow("2",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("2",(int(1280*0.85),int(360*0.85)))
        cv2.moveWindow("2",500,400)
        cv2.putText(gray_extracted_image, "grayscale", (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (77, 255, 9), 2)
        cv2.putText(threshold_img, "binary", (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (77, 255, 9), 2)
        cv2.imshow("2",np.hstack([gray_extracted_image,threshold_img]))

        cv2.waitKey(50)




if __name__ == '__main__':
    main()
    print("Close Target Detection Publisher")

print("-------------OKAY-------------")