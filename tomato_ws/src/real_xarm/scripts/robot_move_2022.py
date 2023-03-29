#!/usr/bin/env python3

import math
from math import radians,degrees
import sys
import random
import copy
import time
import numpy as np
# from tomato_utils.tomato_motors import TomatoDynamixelSDK
from tomato_utils.tcp_client import DollyClient

import struct
import ctypes

import rospy
#from sensor_msgs import point_cloud2
#from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Image
#import geometry_msgs.msg
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import Pose,PoseStamped,PointStamped,PoseArray
from std_msgs.msg import String
#from visualization_msgs.msg import Marker


import tf
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo
import moveit_commander
from tf.transformations import quaternion_from_euler,euler_from_quaternion

from tomato_detection.srv import SelectTomato
"""
    This file is for commad the robot to move in 2022
"""


class xarm7_move():
    def __init__(self):
        #For msg
        self.point_pub = rospy.Publisher('/closetTomato_World', PointStamped, queue_size=1)
        self.xarm2dolly_pub = rospy.Publisher('/xarm_dolly', String, queue_size=1)
        rospy.Subscriber('/rgb/camera_info',CameraInfo,self.camInfoCallback)
        rospy.Subscriber('tomatoArray',PoseArray,self.pose_array_callback)
        rospy.Subscriber('graspArray',PoseArray,self.grasp_array_callback)
        rospy.Subscriber('robot_state', String, self.robot_state_callback)
        self.camHeader = Header()
        # For moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "xarm7"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group.set_end_effector_link("vacum_eef")
        self.group.set_max_velocity_scaling_factor(1.0)
        #self.addobject = AddObject()
        #self.addobject.add_block_plane(1.2,3.2,0.4)
        self.ikSolver = ik_solver = IK("link_base","vacum_eef")

        ##
        # self.tomatoDynamixel = TomatoDynamixelSDK()
        # self.tomatoDynamixel.set_speed(4,value=100)
        # rospy.sleep(0.5)
        self.tomatoClient = DollyClient()
        connect_status = self.tomatoClient.connect()
        rospy.logwarn(f"{connect_status}")



        # Config home pose (get from joint states topic)
        self.joint_init = [-5.480953404912725e-05, -7.85610027378425e-05, 4.387526132632047e-05, -6.460847998823738e-06, -6.404148007277399e-05, -1.876341957540717e-05, 6.65311308694072e-05]
        self.joint_holdUp = [2.924433111228808e-05, 0.00011377803761547511, -7.086276901535626e-05, 7.04990536704031e-05, -0.0001107589616342608, -1.5707110973736906, 0.0001156782648976673]
        self.joint_tomato_init = [0.4421803951263428, 0.12267749011516571, 0.38958969712257385, 0.18971261382102966, 0.913192629814148, -0.7473722696304321, -0.1281263381242752]
        self.joint_b4_pre_pick = [1.4383487701416016, 0.07301218062639236, -0.2581051290035248, 0.30144160985946655, 1.2086621522903442, -1.454795002937317, -0.2397182136774063]
        #self.joint_tomato_pre_drop =[-1.4560242891311646, -0.1350332349538803, -0.004766951780766249, 0.2642664313316345, -1.4874916076660156, -1.4834187030792236, 0.39775633811950684]
        self.joint_tomato_drop = [1.6407618522644043, -0.18750205636024475, -0.30255362391471863, 0.6673380732536316, 2.553243398666382, -0.8784024715423584, -1.405264973640442] #[-0.8232962489128113, -0.37254276871681213, -0.32760536670684814, 0.2948424816131592, -1.9932690858840942, -0.6389274597167969, 1.0421240329742432]
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #Add collision plane
        rWall_pose = PoseStamped()
        rWall_pose.header.frame_id = "link_base"
        rWall_pose.pose.position.y = -0.165
        rWall_pose.pose.position.z = 0.25
        rWall_name = "rWall"
        self.scene.add_box(rWall_name, rWall_pose, size=(0.3, 0.01, 0.3+0.04))

        lWall_pose = PoseStamped()
        lWall_pose.header.frame_id = "link_base"
        lWall_pose.pose.position.y = 0.5
        lWall_name = "lWall"
        self.scene.add_box(lWall_name, lWall_pose, size=(1.0, 0.01, 1.0))

        bWall_pose = PoseStamped()
        bWall_pose.header.frame_id = "link_base"
        bWall_pose.pose.position.z = -0.015
        bWall_name = "bWall"
        self.scene.add_box(bWall_name, bWall_pose, size=(0.3, 0.3, 0.01))
        
        self.robotState = "waiting"
        self.tomatoCenterArray = []
        self.graspArray = []
        self.maxNumberPerBranches = 5
        self.isFinishBranch = 0
    
    def camInfoCallback(self,msg):
         self.camHeader = msg.header

    def robot_state_callback(self,msg):
         self.robotState = msg.data
    
    def pose_array_callback(self,msg):
        tomatoPoseArray = [] 
        for pose in msg.poses:
            tomatoPos = [pose.position.x,pose.position.y,pose.position.z]
            tomatoRot = list(euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))
            
            tomatoPose = tomatoPos+ self.heta_action_from_rotation(tomatoRot)
            tomatoPoseArray.append(tomatoPose)
        self.tomatoCenterArray = tomatoPoseArray #
    
    def grasp_array_callback(self,msg):
        tomatoPoseArray = [] 
        for pose in msg.poses:
            tomatoPos = [pose.position.x,pose.position.y,pose.position.z]
            tomatoRot = list(euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))
            
            tomatoPose = tomatoPos+ self.heta_action_from_rotation(tomatoRot)
            tomatoPoseArray.append(tomatoPose)
        self.graspArray = tomatoPoseArray #
    
    def heta_action_from_rotation(self,rotation3):
        sumrotation2rotateDict = {-6: -1,
                                    5:0,
                                    4: 1}
        sumRotation = int(sum(rotation3))
        #print(f"tomatorot is {sumRotation}")
        #print("sumRotation",sumRotation)
        robotRotation = sumrotation2rotateDict[sumRotation]
        return [robotRotation]

    def publishPoint3(self,pos3):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "world"

        point.point.x = pos3[0]
        point.point.y = pos3[1]
        point.point.z = pos3[2]
        self.point_pub.publish(point)

    def executeJoint(self,jointStates,isWait = True): 
        
        jointGoal = jointStates
        self.group.go(jointGoal,wait=isWait)
        self.group.stop()
    
    def execute_custom_joint(self,joint_nums = [1,2],degrees = [0,0]): 
        current_joint = self.group.get_current_joint_values()
        for num,update_joint in enumerate(joint_nums):
            current_joint[update_joint-1] += math.radians(degrees[num]) #-1 for index 0
        self.executeJoint(current_joint)
    
    def executeWaypoint(self,goal_pose,isWait=True):
        waypoints = [] 
        wpose = goal_pose
        print(wpose)
        waypoints.append(copy.deepcopy(wpose))
        for i in range (4):
            (plan, fraction) = self.group.compute_cartesian_path(
                    waypoints,   # waypoints to follow
                    0.01,        # eef_step
                    2)         # jump_threshold

        if (plan is None) :
            return
        for i in range (3):
            execute = self.group.execute(plan, wait=isWait)
            # print ('execute pick : ', execute)
            if execute :
                break
    
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
    
    def execute_path(self,goal_poses,isWait=True):
        waypoints = copy.deepcopy(goal_poses)
        for i in range (4):
            (plan, fraction) = self.group.compute_cartesian_path(
                    waypoints,   # waypoints to follow
                    0.01,        # eef_step
                    2)         # jump_threshold

        if (plan is None) :
            return
        for i in range (3):
            execute = self.group.execute(plan, wait=isWait)
            # print ('execute pick : ', execute)
            if execute :
                break
    
    def get_path_from(self,pos_list,euler_rotation = [3.14159, -1.5708, 0],frameIn = 'rgb', frameOut ='world'): 
        
        poses = []
        for xyz_position in pos_list:
            goal= Pose()
            goal.position.x = xyz_position[0]
            goal.position.y = xyz_position[1]
            goal.position.z = xyz_position[2]

            quaternion_rotation = tf.transformations.quaternion_from_euler(euler_rotation[0], euler_rotation[1], euler_rotation[2])
            goal.orientation.x = quaternion_rotation[0]
            goal.orientation.y = quaternion_rotation[1]
            goal.orientation.z = quaternion_rotation[2]
            goal.orientation.w = quaternion_rotation[3]
            poses.append(goal)
        return poses
       
    def executeGoal(self,pose): 
        
        self.group.set_pose_target(pose, end_effector_link="vacum_eef")
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
    
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
    
    def findAvoidCollidingPosition(self,pickPosition,prePickPosition,hetaAction):
        rotationDict = { -1:-20,
                        0:0,
                        1:20}
        rotation = rotationDict[hetaAction]
        tomatoAlignPos = pickPosition
        tomatoAlignPos[2] = prePickPosition[2] #Aligning on the same plane
        r = math.dist(tomatoAlignPos, prePickPosition)
        rdiff = 0.025
        R = r + rdiff
        updatePrePickPos_world = [  prePickPosition[0] + R*(1-math.cos(radians(rotation))),
                                    prePickPosition[1] - R*math.sin(radians(rotation)),    
                                    prePickPosition[2]]
        # updatePickPose_world = [  prePickPosition[0] + r + rdiff*(1-math.cos(radians(rotation))),
        #                             prePickPosition[1] - rdiff*math.sin(radians(rotation)),    
        #                             prePickPosition[2]]
        updatePickPose_world = [  pickPosition[0] + rdiff*(1-math.cos(radians(rotation))),
                                    pickPosition[1] - rdiff*math.sin(radians(rotation)),    
                                    pickPosition[2]]
        # updatePickPose_world = [  pickPosition[0] + rdiff,
        #                             pickPosition[1],    
        #                             pickPosition[2]]
        #prePickPose_world = self.get_pose_from(updatePrePickPos_world,euler_rotation = [3.14159, -1.5708, radians(rotation)])
        #self.waypoint_execute(prePickPose_world)
        return updatePrePickPos_world,updatePickPose_world,rotation 

    def executePick(self,tomatoPose):
        # self.tomatoDynamixel.enable_motor(2)
        time.sleep(0.1)
        #self.executeJoint(self.joint_b4_pre_pick,isWait=True)
        #time.sleep(1)
        pickPos3 = tomatoPose[:3] #x y z 
        hetaAction  = tomatoPose[3:][0] #flatten last item
        print(f"Zrot:{hetaAction}")
        len_finger = 0.115
        tomatoPickPos_world = [pickPos3[0] + 0.02,
                                pickPos3[1]+0.0,    
                                pickPos3[2] + 0.00]  #gripper Compensate
        tomato_calibrate_pos_world = [tomatoPickPos_world[0] - 0.10 -len_finger,
                                    tomatoPickPos_world[1],    
                                    tomatoPickPos_world[2]]  #gripper Compensate
        #tomato_calibrate_pose = self.get_pose_from(tomato_calibrate_pos_world)
        #self.executeWaypoint(tomato_calibrate_pose)
        #time.sleep(3)
        # testSrv = self.findTomatoClient([0,0,0],"find_tomato_offset")
        # print(f"Test:{testSrv}")
        # input("Enter to continue")
        #return

        tomatoPrePickPos_world = [tomatoPickPos_world[0] - 0.08 -len_finger,
                            tomatoPickPos_world[1],    
                            tomatoPickPos_world[2]-0.00]  
        rotatedPrePickPos,rotatedPickPos,rotation = self.findAvoidCollidingPosition(tomatoPickPos_world,tomatoPrePickPos_world,hetaAction)
        self.publishPoint3(tomatoPrePickPos_world)
        #time.sleep(1)
        tomatoPrePickPose = self.get_pose_from(tomatoPrePickPos_world)
        tomatoPrePickPath = self.get_path_from([tomato_calibrate_pos_world,tomatoPrePickPos_world])
        print(tomatoPrePickPath)
        self.execute_path(tomatoPrePickPath)
        #self.executeWaypoint(tomatoPrePickPose)
        # enter2Continure = input("Enter2Continue")
        # if enter2Continure == "q":
        #     exit()
        # self.tomatoDynamixel.execute("v1g0")
        time.sleep(0.5)
        after_angle = 0
        if rotation != 0:
            self.publishPoint3(rotatedPrePickPos)
            #time.sleep(1)
            #rotatedPrePickPose = self.get_pose_from(rotatedPrePickPos,euler_rotation = [3.14159, -1.5708, radians(rotation)])
            #self.executeWaypoint(rotatedPrePickPose)
        
            self.publishPoint3(rotatedPickPos)
            #time.sleep(1)
            #rotatedPickPose = self.get_pose_from(rotatedPickPos,euler_rotation = [3.14159, -1.5708, radians(rotation)])
            #self.executeWaypoint(rotatedPickPose)
            rotatedPickPos_L = [tomatoPickPos_world[0]-0.01 -len_finger,
                            tomatoPickPos_world[1]-0.01,    
                            tomatoPickPos_world[2]]
            rotatedPickPos_R = [tomatoPickPos_world[0]-0.01 -len_finger,
                            tomatoPickPos_world[1]+0.01,    
                            tomatoPickPos_world[2]]
            rotatedPickPos_ex = rotatedPickPos_R if rotation > 0 else rotatedPickPos_L
            # rotatePickPath = self.get_path_from([rotatedPrePickPos,rotatedPickPos,rotatedPickPos_ex],euler_rotation = [3.14159, -1.5708, radians(rotation)])
            rotatePickPath = self.get_path_from([rotatedPrePickPos,rotatedPickPos],euler_rotation = [3.14159, -1.5708, radians(rotation)])
            ##########grip############
            self.execute_path(rotatePickPath)
            tomatoPrePickPos_world = rotatedPrePickPos
            tomatoPickPos_world = rotatedPickPos
            after_angle = radians(rotation)
        else:
            self.publishPoint3(tomatoPickPos_world)
            #time.sleep(1)
            #PickPose = self.get_pose_from(tomatoPickPos_world)
            #self.executeWaypoint(PickPose)
            tomatoPickPos_world_R = [tomatoPickPos_world[0] -len_finger,
                            tomatoPickPos_world[1]-0.01,    
                            tomatoPickPos_world[2]+0.0]
            tomatoPickPos_world_L = [tomatoPickPos_world[0] -len_finger,
                            tomatoPickPos_world[1]+0.005,    
                            tomatoPickPos_world[2]+0.0]
            pickPath = self.get_path_from([tomatoPickPos_world_R,tomatoPickPos_world])
            self.execute_path(pickPath)
        time.sleep(0.7)
        
        tomatoLiftPos_world = [tomatoPickPos_world[0]-0.04 - len_finger,
                            tomatoPickPos_world[1],    
                            tomatoPickPos_world[2]+0.01]
        self.publishPoint3(tomatoLiftPos_world)
        #time.sleep(1)
        #time.sleep(1)
        tomatoLiftPose = self.get_pose_from(tomatoLiftPos_world,euler_rotation = [3.14159, -1.5708, after_angle])
        # self.executeWaypoint(tomatoLiftPose)
        # self.tomatoDynamixel.execute("v1g3")
        time.sleep(0.5)        
        # self.tomatoDynamixel.execute("v0g3")
        time.sleep(0.5)
        tomatoHalfCutPos_world = [tomatoLiftPos_world[0]-0. -len_finger,
                            tomatoLiftPos_world[1],    
                            tomatoLiftPos_world[2]+0.00]
        self.publishPoint3(tomatoHalfCutPos_world)
        #time.sleep(1)                    
        tomatoCutPose = self.get_pose_from(tomatoHalfCutPos_world,euler_rotation = [3.14159, -1.5708, after_angle])
        
        
        #self.executeWaypoint(tomatoCutPose)
        # self.tomatoDynamixel.execute("v1g3")
        # time.sleep(1.0)
        # self.tomatoDynamixel.execute("v0g3")
        # time.sleep(0.1)

        # self.execute_custom_joint(joint_nums=[5,6],degrees=[-20,20])
        #self.execute_custom_joint(joint_nums=[5,6],degrees=[20,-20])
        #time.sleep(0.25)
        #self.execute_custom_joint(joint_num=5,degree=-25)
        #self.execute_custom_joint(joint_num=5,degree=25)
        #time.sleep(0.25)
        #self.execute_custom_joint(joint_num=6,degree=-18)
        #time.sleep(0.25)
        
        
        
    def search_tomato(self):
        # sendResult = self.tomatoClient.sendMessage(data="Move")
        # print(sendResult)
        time.sleep(0.2)
        closet_tomato_y_campos = 1000
        tomatoArray = []
        dolly_speed = 0.08 * 1.2
        xarm_y_pos = 0.24
        while abs(closet_tomato_y_campos-xarm_y_pos) > 0.4:
            tomatoArray = self.tomatoCenterArray
            closet_tomato_y_campos = tomatoArray[0][1] if len(tomatoArray) > 0 else 1000
        # if abs(closet_tomato_y_campos-xarm_y_pos) < 0.2:
        #     print(f"found tomato at {closet_tomato_y_campos}")
        #     sendResult = self.tomatoClient.sendMessage(data="Stop")
        #     print(sendResult)
        # else:
        time.sleep(abs(closet_tomato_y_campos-xarm_y_pos)/dolly_speed)
        print(f"found tomato at {closet_tomato_y_campos}")
        # sendResult = self.tomatoClient.sendMessage(data="Stop")
        # print(sendResult)
        
        # startPickMsg = self.tomatoClient.waitForMessage()
        # rospy.logwarn(f'startPickMessage: {startPickMsg}') 

    def initialize_pick_loop(self):
        # for tomatoByDistanceIndex in range(len(self.tomatoCenterArray)):
        #     print(self.tomatoCenterArray)
        numberOfTomato = len(self.graspArray)
        
        if numberOfTomato < 1 or not self.isFinishBranch:
            self.search_tomato() #First time use the tomato stop array
        self.isFinishBranch = 0
        for i in range(self.maxNumberPerBranches): 
            rospy.logwarn(f"ini number of tomato is {len(self.graspArray)}")
            checkCount = 0
            currentGraspArray = self.graspArray
            while checkCount <= 20:
                if len(currentGraspArray) > 0:
                    break
                checkCount += 1
                currentGraspArray = self.graspArray
                time.sleep(0.05)
            if checkCount >= 9:
                return 0
            tomatoCenterPose_cam = currentGraspArray[0]
            print(tomatoCenterPose_cam)
            #### Check Stable ####
            checkInterval = 0.5 #sec 
            checkTol = 0.02 # m
            runoutTime = 3 #sec
            startTime = time.time()
            while time.time() - startTime < runoutTime:
                print("Waiting for stable Tomamto")
                time.sleep(checkInterval)
                if len(self.graspArray) <= 0:
                    continue
                checkStablePose_cam = self.graspArray[0]
                print(checkStablePose_cam)
                if math.dist(checkStablePose_cam[0:3],tomatoCenterPose_cam[0:3]) < checkTol:
                    print("break")
                    print(tomatoCenterPose_cam)
                    print(tomatoCenterPose_cam)
                    print(math.dist(checkStablePose_cam,tomatoCenterPose_cam))
                    break
                else:
                    tomatoCenterPose_cam = checkStablePose_cam
                    print(f"Tomato delta = {math.dist(checkStablePose_cam,tomatoCenterPose_cam)}")
            ########
            print(tomatoCenterPose_cam,'tomatoCenterPos_cam')
            tomatoCenterPos3_world = self.transformPoint3(tomatoCenterPose_cam[:3],frameIn=self.camHeader.frame_id,frameOut="world")
            print(tomatoCenterPos3_world,'tomatoCenterPos_world')
            tomatoCenterPose_world = tomatoCenterPos3_world + list(tomatoCenterPose_cam[3:])
            self.publishPoint3(tomatoCenterPose_world[:3])
            self.executePick(tomatoCenterPose_world)
            
            self.executeJoint(self.joint_tomato_init)
            
            # self.tomatoDynamixel.enable_motor(4)
            # time.sleep(0.1)
            # self.tomatoDynamixel.execute("v0g0")
            # time.sleep(0.2)
            # self.tomatoDynamixel.disable_motor(2)
        return 0

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
        self.executeJoint(self.joint_holdUp)
        # self.tomatoDynamixel.execute("v0g0")
        #time.sleep(1)
        command = input("Press any key to continue: ")
        if command == "q":
            exit()
        count = 0
        
        startPickMsg = ""
        while not rospy.is_shutdown() and self.robotState != "stop":
            #startPickMsg = self.tomatoClient.waitForMessage()
            print("Start Picking")
            numberOfTomatoLeft = self.initialize_pick_loop()
            #self.xarm_dolly_pub.publish("finish_pick")


            # print("v0g0c0")
            # self.tomatoDynamixel.execute("v0g0c0")
            # time.sleep(2)
            # print("v1g0c0")
            # self.tomatoDynamixel.execute("v1g0c0")
            # time.sleep(2)
            # print("v1g1c0")
            # self.tomatoDynamixel.execute("v1g1c0")
            # time.sleep(2)
            # print("v0g1c0")
            # self.tomatoDynamixel.execute("v0g1c0")
            # time.sleep(2)
            # print("v0g1c1")
            # self.tomatoDynamixel.execute("v0g1c1")
            # time.sleep(2)
            # print("v0g0c0")
            # self.tomatoDynamixel.execute("v0g0c0")
            # time.sleep(2)
            

if __name__ == '__main__':
    try:
        rospy.init_node('spaghetti_grasping_rgbd_execute', anonymous=True)
        _xarm7_move = xarm7_move()
        _xarm7_move.process()
    except rospy.ROSInterruptException:
        pass
 