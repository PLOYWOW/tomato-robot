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
from simple_detection.scripts.utils.rs_process_ver2 import rs_process_color
from utils import gui_creation


# color_w = int(rospy.get_param('/color_width'))
color_w = int(1280)
# color_h = int(rospy.get_param('/color_height'))
color_h = int(720)
camera_resolution_color = {'rgb-w':color_w,'rgb-h':color_h,}

rs_image_rgb = np.zeros((camera_resolution_color['rgb-w'],camera_resolution_color['rgb-h'],3), np.uint8)

#target detect args
enable_image_show = True
enable_fps_show   = True

def main():
    rospy.init_node('hand_tracking', anonymous=True)
    rospy.loginfo("--- Target Detection Start!")

    rsProcess_color = rs_process_color(camera_resolution_color)

    #load the param
    rs_img_name = 'RS_SETTING'
    type_name1 = 'mask1--'
    type_name2 = 'mask2--'

    #time for fps calculation
    start_time = datetime.datetime.now()
    num_frames = 0

    while not rospy.is_shutdown():
        #get rgb,depth frames for synchronized frames
        if not rsProcess_color.isOK: #not False -> True
            continue

        rsProcess_color.isOK = False

        #start processiog
        img_bgr = rsProcess_color.bgr_image 
        img_rgb = rsProcess_color.rgb_image

        # Remove background - Set pixels further than clipping_distance to grey
        black_color = 0

        cv2.imshow("RGB image", img_rgb)
        cv2.imshow("BGR image", img_bgr)
        cv2.waitKey(50)
        
if __name__ == '__main__':
    main()
    print("Close Target Detection Publisher")