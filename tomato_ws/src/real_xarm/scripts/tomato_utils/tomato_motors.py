#!/usr/bin/env python

import sys
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand,DynamixelCommandRequest
from tomato_utils import PyDynamixel_v2 as pd

class TomatoDynamixels():
    def __init__(self) -> None:
        self.angle_command_req = DynamixelCommandRequest()
        self.dynamixel_command_service = rospy.ServiceProxy('dynamixel_workbench/dynamixel_command',DynamixelCommand)
        self.command_to_angles = {"v0g0":[90,90],"v1g0":[85,90],"v0g1":[90,180],"v1g1":[85,180],"v1g2":[85,160]}
        self.motor_ids = [1,4]#Hand Vacum

    def execute(self,command="v0g0"):
        angles = self.command_to_angles[command]
        results = [self.send_angle(angle,id) for angle,id in zip(angles, self.motor_ids)]
        return results

    def send_angle(self,angle,id):
        angle_command_req = DynamixelCommandRequest()
        angle_command_req.id = id
        angle_command_req.addr_name = 'Goal_Position'
        angle_command_req.value = self.degree_to_joint_value(angle,id)

        rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
        try:
            result = self.dynamixel_command_service(angle_command_req)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def degree_to_joint_value(self,angle,id,offset = 0):
        if id == 1:
            return int(1024.0*angle/300) + offset
        if id == 4:
            return int(2048.0*angle/180) + offset 

class TomatoDynamixelSDK():
    def __init__(self) -> None:
        # Declare the DxlComm variable
        port = "/dev/ttyUSB0"
        baudrate = 57600
        self.serial = pd.DxlComm(port=port, baudrate=baudrate)
        # Declare a dynamixel joint
        self.mortor_list = pd.Joint(4,max_value=1024,max_angle=300),pd.Joint(2,max_value=2048,max_angle=180)#,pd.Joint(9,max_value=1024,max_angle=300) #vacum,gripper,cutter
        self.serial.attach_joints(self.mortor_list)
        # # You could also send all joints as a list to DxlComm varible
        # self.serial.attach_joints([vacum, gripper, cutter])
        self.command_to_angles = {"v0g0":[90,160],"v1g0":[80,160],"v1g1":[80,223],"v1g2":[80,223],"v1g3":[80,223],"v0g3":[90,223]} #g1 180
        self.ID2IndexDict = {self.mortor_list[0].servo_id:0,self.mortor_list[1].servo_id:1}
    
    
    def execute(self,command="v0g0"):
        angles = self.command_to_angles[command]
        results = [motor.send_angle(angle) for angle,motor in zip(angles, self.mortor_list)]
        return results

    def enable_motor(self,id):
        enable_id = self.ID2IndexDict[id]
        self.mortor_list[enable_id].enable_torque()

    def disable_motor(self,id):
        disable = self.ID2IndexDict[id]
        self.mortor_list[disable].disable_torque()
    
    def set_speed(self,id,value=100):
        set_speed_id = self.ID2IndexDict[id]
        self.mortor_list[set_speed_id].send_profile_velocity(value)
    

    
    
    