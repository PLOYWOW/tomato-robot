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
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import Pose,PoseStamped,PointStamped
#from std_msgs.msg import Float64
#from visualization_msgs.msg import Marker


import tf
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
import moveit_commander

from tomato_detection.srv import SelectTomato
"""
    This file is for commad the robot to move 
"""


class xarm7_move():
    def __init__(self):
        #For msg
        self.point_pub = rospy.Publisher('/closetTomato_World', PointStamped, queue_size=1)
        rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.camInfoCallback)
        self.camHeader = Header() 
        # For moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "xarm7"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group.set_end_effector_link("vacum_eef")
        self.group.set_max_velocity_scaling_factor(0.2)
        #self.addobject = AddObject()
        #self.addobject.add_block_plane(1.2,3.2,0.4)
        self.ikSolver = ik_solver = IK("link_base","vacum_eef")
        rospy.sleep(2)



        # Config home pose (get from joint states topic)
        self.joint_init = [-5.480953404912725e-05, -7.85610027378425e-05, 4.387526132632047e-05, -6.460847998823738e-06, -6.404148007277399e-05, -1.876341957540717e-05, 6.65311308694072e-05]
        self.joint_holdUp = [2.924433111228808e-05, 0.00011377803761547511, -7.086276901535626e-05, 7.04990536704031e-05, -0.0001107589616342608, -1.5707110973736906, 0.0001156782648976673]
        self.joint_tomato_init = [-0.8844854235649109, 0.012783500365912914, -0.6170864105224609, 0.018508732318878174, -1.501579761505127, -1.5773736238479614, 0.00838994886726141]
        self.joint_tomato_ready = [-0.5185768008232117, 0.14321313798427582, -0.2335641235113144, 0.24865911900997162, -0.7509441375732422, -1.5129140615463257, 0.09866613894701004]
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #Add collision plane
        rWall_pose = PoseStamped()
        rWall_pose.header.frame_id = "link_base"
        rWall_pose.pose.position.y = 0.5
        rWall_name = "rWall"
        self.scene.add_box(rWall_name, rWall_pose, size=(1.0, 0.01, 1.0))

        lWall_pose = PoseStamped()
        lWall_pose.header.frame_id = "link_base"
        lWall_pose.pose.position.y = -0.5
        lWall_name = "lWall"
        self.scene.add_box(lWall_name, lWall_pose, size=(1.0, 0.01, 1.0))
        self.robot
    
    def camInfoCallback(self,msg):
         self.camHeader = msg.header

    def publishPoint3(self,pos3):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "world"

        point.point.x = pos3[0]
        point.point.y = pos3[1]
        point.point.z = pos3[2]
        self.point_pub.publish(point)

    def executeJoint(self,jointStates): 
        
        jointGoal = jointStates
        self.group.go(jointGoal,wait=True)
        self.group.stop()
    
    def executeWaypoint(self,goal_pose):
        
        waypoints = [] 
        wpose = goal_pose
        print(wpose)
        
        waypoints.append(copy.deepcopy(wpose))
        
        
        for i in range (4):
            (plan, fraction) = self.group.compute_cartesian_path(
                    waypoints,   # waypoints to follow
                    0.01,        # eef_step
                    3)         # jump_threshold

        if (plan is None) :
            return

        for i in range (3):
            execute = self.group.execute(plan, wait=True)
            # print ('execute pick : ', execute)
            if execute :
                break
    def executePath(self,joint_poses):

        for joint in joint_poses: 
            joint_goal = joint
            self.group.go(joint_goal,wait=True)
            self.group.stop()
   
    def executeGoal(self,pose): 
        
        self.group.set_pose_target(pose, end_effector_link="vacum_eef")
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
    
    

    def solveIK(self,xyz_position,euler_rotation = [3.14159, -1.5708, 0]):
        quat4 = tf.transformations.quaternion_from_euler(euler_rotation[0], euler_rotation[1], euler_rotation[2])
        seed_state = [0.0] * self.ikSolver.number_of_joints
        joint_states = None

        start_time = time.time()
        while time.time() - start_time < 3:
            joint_states = self.ikSolver.get_ik(seed_state,
                            xyz_position[0],xyz_position[1],xyz_position[2],  # X, Y, Z
                            quat4[0], quat4[1], quat4[2], quat4[3])
                            #brx=math.radians(30), bry=math.radians(30),brz=math.radians(30))  # QX, QY, QZ, Qw
            if joint_states is None:
                rospy.logwarn("Cannnot file IK Solution Trying Again")
            else:
                rospy.logwarn("Cannnot file IK Solution Getting out")
                break
        return list(joint_states)

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
        r = math.dist(tomatoAlignPos, prePickPosition)
        rdiff = 0.03
        R = r + rdiff
        updatePrePickPos_world = [  prePickPosition[0] + R*(1-math.cos(radians(rotation))),
                                    prePickPosition[1] - R*math.sin(radians(rotation)),    
                                    prePickPosition[2]]
        updatePickPose_world = [  prePickPosition[0] + r + rdiff*(1-math.cos(radians(rotation))),
                                    prePickPosition[1] - rdiff*math.sin(radians(rotation)),    
                                    prePickPosition[2]]
        #prePickPose_world = self.get_pose_from(updatePrePickPos_world,euler_rotation = [3.14159, -1.5708, radians(rotation)])
        #self.waypoint_execute(prePickPose_world)
        return updatePrePickPos_world,updatePickPose_world,rotation 

    def executePick(self,tomatoPose):
        self.executeJoint(self.joint_holdUp)
        pickPos3 = tomatoPose[:3] #x y z 
        hetaAction  = tomatoPose[3:][0] #flatten last item
        print(f"Zrot:{hetaAction}")
        tomatoPickPos_world = [pickPos3[0]+0.00,
                                pickPos3[1],    
                                pickPos3[2]+0.01]  #gripper Compensate

        tomatoPrePickPos_world = [tomatoPickPos_world[0] - 0.05,
                            tomatoPickPos_world[1],    
                            tomatoPickPos_world[2]-0.00]  
        rotatedPrePickPos,rotatedPickPos,rotation = self.findAvoidCollidingPosition(pickPos3,tomatoPrePickPos_world,hetaAction)
        if rotation != 0:
            tomatoPickPos_world[0] += 0.01
        self.publishPoint3(tomatoPickPos_world)
        tomatoPrePickPose = self.get_pose_from(tomatoPrePickPos_world)
        PrePickJoints = self.solveIK(tomatoPrePickPos_world)
        #self.executeJoint(PrePickJoints)
        #print(PrePickJoints)
        self.executeWaypoint(tomatoPrePickPose)
        #time.sleep(1)
        if rotation != 0:
            self.publishPoint3(rotatedPrePickPos)
            #rotatedPrePickJoints = self.solveIK(rotatedPrePickPos,euler_rotation = [3.14159, -1.5708, radians(rotation)])
            rotatedPrePickPose = self.get_pose_from(rotatedPrePickPos,euler_rotation = [3.14159, -1.5708, radians(rotation)])
            self.executeWaypoint(rotatedPrePickPose)
            
            self.publishPoint3(rotatedPickPos)
            #rotatedPrePickJoints = self.solveIK(rotatedPrePickPos,euler_rotation = [3.14159, -1.5708, radians(rotation)])
            rotatedPickPose = self.get_pose_from(rotatedPickPos,euler_rotation = [3.14159, -1.5708, radians(rotation)])
            self.executeWaypoint(rotatedPickPose)
            #self.executeJoint(rotatedPrePickJoints)
            #time.sleep(1)
            #self.publishPoint3(tomatoPickPos_world)
            #PickPose = self.get_pose_from(tomatoPickPos_world,euler_rotation = [3.14159, -1.5708, radians(rotation)])
            #self.executeWaypoint(PickPose)
            #time.sleep(1)
        else:
            #time.sleep(1)

            #pickJoints = self.solveIK(tomatoPickPos_world)
            #self.executeJoint(pickJoints)
            self.publishPoint3(tomatoPickPos_world)
            PickPose = self.get_pose_from(tomatoPickPos_world)
            self.executeWaypoint(PickPose)
            #print(PickJoints)
            #time.sleep(1)
        self.executeWaypoint(tomatoPrePickPose)
            
        

        


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
        self.executeJoint(self.joint_tomato_init)
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
            gripperPos_cam = self.transformPoint3(gripperPos_world,frameIn="world",frameOut=self.camHeader.frame_id)
            tomatoCenterPose_cam = self.findTomatoClient(gripperPos_cam,"closet_tomato")
            while sum(tomatoCenterPose_cam) == 0:
                tomatoCenterPose_cam = self.findTomatoClient(gripperPos_cam,"closet_tomato")
            print(tomatoCenterPose_cam,'tomatoCenterPos_cam')
            tomatoCenterPos3_world = self.transformPoint3(tomatoCenterPose_cam[:3],frameIn=self.camHeader.frame_id,frameOut="world")
            print(tomatoCenterPose_cam)
            tomatoCenterPose_world = tomatoCenterPos3_world + list(tomatoCenterPose_cam[3:])
            print(tomatoCenterPose_world,"FAR")
            self.publishPoint3(tomatoCenterPose_world[:3])
            self.executePick(tomatoCenterPose_world)
            self.executeJoint(self.joint_tomato_init)

            

if __name__ == '__main__':
    try:
        rospy.init_node('spaghetti_grasping_rgbd_execute', anonymous=True)
        _xarm7_move = xarm7_move()
        _xarm7_move.process()
    except rospy.ROSInterruptException:
        pass
 