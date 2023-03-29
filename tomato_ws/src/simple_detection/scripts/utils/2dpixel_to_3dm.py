from scipy.interpolate import InterpolatedUnivariateSpline
import numpy as np
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker
import rospy
#############################################
# Convert tomato center pt. 2D(px) -> 3D(m)
#############################################
####### 3 Input from yolo
# Detected area(px2)
area_yolo = 330000
# Detected center x(px)
u = 600
# Detected center y(px)
v = 300

####### Interpolate depth (ref w/ minicam) (in m) from detected area (in pixel2)
# Collected data #depth & tomato size (area)
z = np.array([0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.20,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.30])
z = np.sort(z)
z = z[::-1]
area = np.array([424032,336736,315525,280000,262752,229270,202776,186126,160763,149328,130419,127088,116675,110664,98936,91126,81928,75301,71250,67344])
area = np.sort(area)
# Interpolate function
spl = InterpolatedUnivariateSpline(area, z, k=1, check_finite=False)
# output depth in m
z_yolo = spl(area_yolo) 

####### Minicam intrinsic parameters
fx = 1408.804187
fy = 1413.974767
ppx = 578.154935
ppy = 372.542587

####### Parameters for converting 2D(px) -> 3D(m)
# u: detected center x (px)
# v: detected center y (px)
# z_yolo: interpolated depth (m)
# fx, fy: focal length
# ppx, ppy: principal pt. offset

####### Convert tomato center pt. 2D(px) -> 3D(m)
x = ((u-ppx)*z_yolo)/fx
y = ((v-ppy)*z_yolo)/fy

####### Detected center in m
# x (m)
# y (m)
# z_yolo (m)

marker_pub = rospy.Publisher("/minicam_marker", Marker, queue_size = 2)

marker = Marker()

marker.header.frame_id = "/minicam"
# marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker.type = 2
marker.id = 0

# Set the scale of the marker
# marker.scale.x = 1.0
# marker.scale.y = 1.0
# marker.scale.z = 1.0

# Set the color
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0

# Set the pose of the marker
marker.pose.position.x = x
marker.pose.position.y = y
marker.pose.position.z = z_yolo

marker_pub.publish(marker)









