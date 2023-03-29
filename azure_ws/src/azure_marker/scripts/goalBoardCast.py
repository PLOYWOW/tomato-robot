#!/usr/bin/env python  
import roslib
import rospy
from math import radians

from std_msgs.msg import Bool,Float32MultiArray
from geometry_msgs.msg import PoseStamped,PointStamped
import tf
import turtlesim.msg

def handle_goal_pose(msg):
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "/goal"


    point.point.x = msg.pose.position.x
    point.point.y = msg.pose.position.y
    point.point.z = msg.pose.position.z
    
    point_pub.publish(point)
    print("sending Goal")

if __name__ == '__main__':

    rospy.init_node('goal_broadcaster')

    point_pub = rospy.Publisher('/goal_point', PointStamped,
                                          queue_size=1)
    rospy.Subscriber('goal',
                     PoseStamped,
                     handle_goal_pose)
    rospy.spin()