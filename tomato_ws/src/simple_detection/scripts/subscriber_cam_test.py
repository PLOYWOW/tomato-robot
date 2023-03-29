#! /usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

class Subscribe:
    def __init__(self):
        self.height = 0
        rospy.init_node("topic_subscriber")
        # self.height = 0
        # self.image_sub = rospy.Subscriber("/depth_to_rgb/camera_info", CameraInfo, callback)
        rospy.Subscriber("/depth_to_rgb/camera_info", CameraInfo, self.callback(self))
        rospy.spin()

    def callback(self, msg):
        # print(1)
        print(msg.height)
        # self.height = msg.height
        # print(self.height)

scb = Subscribe()
# print(scb.callback.height)