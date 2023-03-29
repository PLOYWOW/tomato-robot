#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs

class rs_process():
    # print("##### IN rs_process")
    def __init__(self,resolution):
        
        # print("#####     IN init fn")
        self.bridge = CvBridge()
        image_align_hand_color = "/color/image_raw"
        image_align_hand_depth = "/depth/image_raw"
        # print("rostopic set ")
        # self.image_sub = rospy.Subscriber(image_align_hand_color, Image, self.color_callback, queue_size=1)
        # print("subscriber set")
        self.isOK = False
        # print("isOK set")
        self.depth_image = np.zeros((resolution['depth-w'],resolution['depth-h'],3), np.uint16)
        self.rgb_image = np.zeros((resolution['rgb-w'],resolution['rgb-h'],3), np.uint8)
        self.bgr_image = np.zeros((resolution['rgb-w'],resolution['rgb-h'],3), np.uint8)
        self.image_sub = rospy.Subscriber(image_align_hand_color, Image, self.color_callback, queue_size=1)
        self.image_sub = rospy.Subscriber(image_align_hand_depth, Image, self.depth_callback, queue_size=1)
        # print("init depth matrix")
        # cv2.imshow("Depth Image in INIT", self.depth_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # print("fininshed init")
        # self.rgb_image = np.zeros((resolution['rgb-w'],resolution['rgb-h'],3), np.uint8)
        # self.bgr_image = np.zeros((resolution['rgb-w'],resolution['rgb-h'],3), np.uint8)

    def color_callback(self, data):
        # print("#####     IN color_callback fn")
        self.isOK = True
        # print (data.encoding) #rgb8
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.rgb_image = cv2.rotate(self.rgb_image,cv2.ROTATE_180)
            self.bgr_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
            # print(self.bgr_image.shape)
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        # print("#####     IN depth_callback fn")
        self.isOK = True
        # print(data.encoding) #16uc1
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough") #32FC1
            self.depth_image = self.depth_image*100
            print(self.depth_image.shape)
            # self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            # cv2.imshow("Depth Image",self.depth_image)
            # cv2.waitKey(50)
            # cv2.destroyAllWindows()
        except CvBridgeError as e:
            print(e)
    
    def activate(self):
        print("#####     IN activate")
        # rospy.init_node("topic_subscriber", anonymous=True)
        # rospy.spin()

    def process(self, ):
        while not rospy.is_shutdown():
            rospy.spin()


# if __name__ == "__main__":
#     try:
#         rospy.init_node("topic_subscriber", anonymous=True)
#         camera_resolution_depth = {'depth-w':1280,'depth-h':720}
#         depth_sub = rs_process_depth(camera_resolution_depth)
#         depth_sub.process()

#     except rospy.ROSInterruptException as e:
#         print(e)