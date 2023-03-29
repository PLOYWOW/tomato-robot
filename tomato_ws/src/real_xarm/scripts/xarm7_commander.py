#!/usr/bin/env python3

import math
from math import radians,degrees
import sys
import random
import copy
import time
import numpy as np


import struct
import ctypes

import rospy
#from sensor_msgs import point_cloud2
#from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Image
#import geometry_msgs.msg
from geometry_msgs.msg import Pose,PoseStamped,PointStamped
#from std_msgs.msg import Float64
#from visualization_msgs.msg import Marker


import tf
import tf2_ros
import tf2_geometry_msgs

import moveit_commander

from tomato_detection.srv import SelectTomato
"""
    This file is for commad the robot to move 
"""


class xarm7_move():
    def __init__(self):
        #For msg
        self.point_pub = rospy.Publisher('/closetTomato_World', PointStamped, queue_size=1)

        # For moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "xarm7"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group.set_end_effector_link("vacum_eef")
        self.group.set_max_velocity_scaling_factor(0.3)
        #self.addobject = AddObject()
        #self.addobject.add_block_plane(1.2,3.2,0.4)
        rospy.sleep(2)



        # Config home pose (get from joint states topic)
        self.joint_init = [-5.480953404912725e-05, -7.85610027378425e-05, 4.387526132632047e-05, -6.460847998823738e-06, -6.404148007277399e-05, -1.876341957540717e-05, 6.65311308694072e-05]
        self.pick_home = [2.924433111228808e-05, 0.00011377803761547511, -7.086276901535626e-05, 7.04990536704031e-05, -0.0001107589616342608, -1.5707110973736906, 0.0001156782648976673]
        self.tomato_init = [-9.536154539091513e-05, 0.1584652215242386, 3.2507028663530946e-05, 0.28025001287460327, -5.460389729705639e-05, -1.4490468502044678, 9.819716069614515e-05]
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #Add collision plane
        backWall_pose = PoseStamped()
        backWall_pose.header.frame_id = "link_base"
        backWall_pose.pose.position.x = -0.16
        backWall_name = "backWall"
        self.scene.add_box(backWall_name, backWall_pose, size=(0.01, 1.0, 1.0))
            
    def publishPoint3(self,pos3):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "world"

        point.point.x = pos3[0]
        point.point.y = pos3[1]
        point.point.z = pos3[2]
        self.point_pub.publish(point)

    def backHome(self): 
        
        joint_goal = self.pick_home
        self.group.go(joint_goal,wait=True)
        self.group.stop()
    
    def waypoint_execute(self,goal_pose):
        
        waypoints = [] 
        wpose = goal_pose
        print(wpose)
        
        waypoints.append(copy.deepcopy(wpose))
        
        
        for i in range (4):
            (plan, fraction) = self.group.compute_cartesian_path(
                    waypoints,   # waypoints to follow
                    0.005,        # eef_step
                    0.00)         # jump_threshold

        if (plan is None) :
            return

        for i in range (3):
            execute = self.group.execute(plan, wait=True)
            # print ('execute pick : ', execute)
            if execute :
                break
    def path_execute(self,joint_poses):

        for joint in joint_poses: 
            joint_goal = joint
            self.group.go(joint_goal,wait=True)
            self.group.stop()
   
    def goal_execute(self,pose): 
        
        self.group.set_pose_target(pose, end_effector_link="vacum_eef")
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
    
    

    def trajectory_create(self,goal_pose):


        # 0. Go to the goal
        #self.goal_execute(goal_pose)
        self.waypoint_execute(goal_pose)#(pose,speed)


        # 1. Go back to home position
        #self.backHome()

    def transformPoint3(self,xyz_position,euler_rotation = [3.14159, -1.5708, 0],frameIn = 'rgb', frameOut ='world'): 
        
        goalStamped = PoseStamped()
        goalStamped.pose.position.x = xyz_position[0] 
        goalStamped.pose.position.y = xyz_position[1]
        goalStamped.pose.position.z = xyz_position[2]

        try:
            
            trans = self.tfBuffer.lookup_transform(frameOut, frameIn, rospy.Time(),rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Cant find transform")
            time.sleep(1)
            return
        goalStamped_world = tf2_geometry_msgs.do_transform_pose(goalStamped, trans)

        goal = [0,0,0]
        goal[0] = goalStamped_world.pose.position.x
        goal[1] = goalStamped_world.pose.position.y
        goal[2] = goalStamped_world.pose.position.z
        return goal

    def get_pose_from(self,xyz_position,euler_rotation = [3.14159, -1.5708, 0],frameIn = 'rgb', frameOut ='world'): 
        
        goal= Pose()
    
        goal.position.x = xyz_position[0]
        goal.position.y = xyz_position[1]
        goal.position.z = xyz_position[2]

        quaternion_rotation = tf.transformations.quaternion_from_euler(euler_rotation[0], euler_rotation[1], euler_rotation[2])
        goal.orientation.x = quaternion_rotation[0]
        goal.orientation.y = quaternion_rotation[1]
        goal.orientation.z = quaternion_rotation[2]
        goal.orientation.w = quaternion_rotation[3]

        return goal
    
    def findAvoidCollidingPosition(self,pickPosition,prePickPosition,hetaAction):
        rotationDict = { -1:-30,
                        0:0,
                        1:30}
        rotation = rotationDict[hetaAction]
        tomatoAlignPos = pickPosition
        tomatoAlignPos[2] = prePickPosition[2] #Aligning on the same plane
        R = math.dist(tomatoAlignPos, prePickPosition)
        updatePrePickPos_world = [  prePickPosition[0] + R*(1-math.cos(radians(rotation))),
                                    prePickPosition[1] - R*math.sin(radians(rotation)),    
                                    prePickPosition[2]]
        #prePickPose_world = self.get_pose_from(updatePrePickPos_world,euler_rotation = [3.14159, -1.5708, radians(rotation)])
        #self.waypoint_execute(prePickPose_world)
        return updatePrePickPos_world,rotation 

    def executePick(self,tomatoPose):
        pickPos3 = tomatoPose[:3] #x y z 
        hetaAction  = tomatoPose[3:][0] #flatten last item
        print(f"Zrot:{hetaAction}")
        tomatoPickPos_world = [pickPos3[0]-0.01,
                                pickPos3[1],    
                                pickPos3[2]+0.01]  #gripper Compensate

        tomatoPrePickPos_world = [tomatoPickPos_world[0] - 0.07,
                            tomatoPickPos_world[1],    
                            tomatoPickPos_world[2]]  
        rotatedPrePickPos,rotation = self.findAvoidCollidingPosition(pickPos3,tomatoPrePickPos_world,hetaAction)
        self.publishPoint3(tomatoPrePickPos_world)
        tomatoPrePickPose = self.get_pose_from(tomatoPrePickPos_world)
        self.waypoint_execute(tomatoPrePickPose)
        #time.sleep(1)
        if rotation != 0:
            self.publishPoint3(rotatedPrePickPos)
            rotatedPrePickPose = self.get_pose_from(rotatedPrePickPos,euler_rotation = [3.14159, -1.5708, radians(rotation)])
            self.waypoint_execute(rotatedPrePickPose)
            #time.sleep(1)
            self.publishPoint3(tomatoPickPos_world)
            PickPose = self.get_pose_from(tomatoPickPos_world,euler_rotation = [3.14159, -1.5708, radians(rotation)])
            self.waypoint_execute(PickPose)
            #time.sleep(1)
        else:
            #time.sleep(1)
            self.publishPoint3(tomatoPickPos_world)
            PickPose = self.get_pose_from(tomatoPickPos_world)
            self.waypoint_execute(PickPose)
            #time.sleep(1)
            
        

        


    def findTomatoClient(self,gripperPos3,serviceName):
        rospy.wait_for_service(serviceName)
        try:
            find_tomato = rospy.ServiceProxy(serviceName, SelectTomato)
            resp1 = find_tomato(gripperPos3)
            return resp1.tomatoPos
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        
        
    
    def process(self):
        print("process start")
        self.backHome()
        command = input("Press any key to continue: ")
        if command == "q":
            exit()
        while not rospy.is_shutdown():
            path_history = []
            joint_goal = []
            #print(self.findTomatoClient([0,0,0]))
            #tomatoCenterPos = rospy.wait_for_message("tomato_pick_pos", PointStamped)
            eef = self.group.get_end_effector_link()
            print(eef)
            gripperPose = self.group.get_current_pose()
            gripperPos_world = [gripperPose.pose.position.x,
                                gripperPose.pose.position.y,
                                gripperPose.pose.position.z]
            gripperPos_cam = self.transformPoint3(gripperPos_world,frameIn="world",frameOut="rgb_camera_link")
            tomatoCenterPose_cam = self.findTomatoClient(gripperPos_cam,"closet_tomato")
            while sum(tomatoCenterPose_cam) == 0:
                tomatoCenterPose_cam = self.findTomatoClient(gripperPos_cam,"closet_tomato")
            print(tomatoCenterPose_cam,'tomatoCenterPos_cam')
            tomatoCenterPos3_world = self.transformPoint3(tomatoCenterPose_cam[:3],frameIn="rgb_camera_link",frameOut="world")
            print(tomatoCenterPose_cam)
            tomatoCenterPose_world = tomatoCenterPos3_world + list(tomatoCenterPose_cam[3:])
            print(tomatoCenterPose_world,"FAR")
            self.publishPoint3(tomatoCenterPose_world[:3])
                          
            #prePickPose_world = self.get_pose_from(tomatoPrePickPos_world) # List --> PoseStamped
            self.executePick(tomatoCenterPose_world)

            # print("CollisionCheck")
            # tomatoPickPos_cam = self.transformPoint3(tomatoPickPos_world,frameIn="world",frameOut="camera_color_optical_frame")
            #tomatoCollidingArray = self.findTomatoClient(tomatoPickPos_cam,"hand_collision_tomato")
            #print(tomatoCollidingArray)

            #self.waypoint_execute(prePickPose_world)
            # joint_goal = self.group.get_current_joint_values()
            # path_history.insert(0,joint_goal)
            
            # print(tomatoCollidingArray)
            #if sum(tomatoCollidingArray) != 0.0:
            #    print("avoidColliding")
                #updatePrePickPose = self.avoidColliding(tomatoCenterPos_world,tomatoPrePickPos_world,rotation = 30)
            #     joint_goal = self.group.get_current_joint_values()
            #     path_history.insert(0,joint_goal)
            #     print("SecondCheck")
            #     tomatoCollidingArray = self.findTomatoClient([0,0,0],"hand_find_tomato")
            #     print(tomatoCollidingArray)
            #     if sum(tomatoCollidingArray) != 0.0:
            #         updatePrePickPose = self.avoidColliding(tomatoCenterPos_world,tomatoPrePickPos_world,rotation = 80)
            #         joint_goal = self.group.get_current_joint_values()
            #         path_history.insert(0,joint_goal)
            #         tomatoCollidingArray = self.findTomatoClient([0,0,0],"hand_find_tomato")

            #time.sleep(1)
            # self.path_execute(path_history)
            self.backHome()

            

if __name__ == '__main__':
    try:
        rospy.init_node('spaghetti_grasping_rgbd_execute', anonymous=True)
        _xarm7_move = xarm7_move()
        _xarm7_move.process()
    except rospy.ROSInterruptException:
        pass
 