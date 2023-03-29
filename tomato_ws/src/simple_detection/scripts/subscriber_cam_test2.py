#! /usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

class Subscribe:
    def __init__(self):
        print("in Subsribe: in init fn")
        self.isOK = False
        print("Here1")
        self.sub = rospy.Subscriber("/depth_to_rgb/camera_info", CameraInfo, self.callback, queue_size=1)
        print("Here2")
        self.camerainfo = CameraInfo()
        print("Here3")
        self.height = 0
        print("Here4")

    def callback(self, msg):
        print("in Subsribe: in callback fn")
        self.isOK = True
        self.camerainfo = msg
        self.height = msg.height
        # print(self.camerainfo)
        # print(self.height)

    def activate(self):
        print("in Subsribe: in activate fn")
        rospy.init_node("topic_subscriber", anonymous=True)
        # subscriber = Subscribe()
        rospy.spin()

# if __name__ == "__main__":
#     rospy.init_node("topic_subscriber", anonymous=True)
#     subscriber = Subscribe()
#     rospy.spin()
    