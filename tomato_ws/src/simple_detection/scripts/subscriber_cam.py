#! /usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

def callback(msg):
    print(msg.height)

rospy.init_node("topic_subscriber")
sub = rospy.Subscriber("/depth_to_rgb/camera_info", CameraInfo, callback)
rospy.spin()

# class subscriber_cam:
#     def __init__(self):
#         rospy.init_node("topic_subscriber")
#         self.

#     def 
