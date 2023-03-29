#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand,DynamixelCommandRequest

#class TomatoDynamixels()

def degree_to_joint_value(angle,id,offset = 0):
    if id == 2:
        return int(1024.0*angle/300) + offset
    if id == 4:
        return int(2048.0*angle/180) + offset 


def send_angle(angle,id):
    angle_command_req = DynamixelCommandRequest()
    angle_command_req.id = id
    angle_command_req.addr_name = 'Goal_Position'
    angle_command_req.value = degree_to_joint_value(angle,id)

    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:
        dynamixel_command_service = rospy.ServiceProxy('dynamixel_workbench/dynamixel_command',DynamixelCommand)
        result = dynamixel_command_service(angle_command_req)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def talker():
    rospy.init_node('motor_command', anonymous=True)
    rate = rospy.Rate(60) # 10hz
    while not rospy.is_shutdown():
        print("Enter input in from angle(degree),id")
        angleInput = input().split(",")
        print(angleInput)
        send_angle(float(angleInput[0]),int(angleInput[1]))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass