#!/usr/bin/env python


import PyDynamixel_v2 as pd
from math import pi
from time import sleep

# Declare the DxlComm variable
port = "/dev/ttyUSB0"
baudrate = 57600
serial = pd.DxlComm(port=port, baudrate=baudrate)

# Declare a dynamixel joint
vacum,gripper,cutter = pd.Joint(2),pd.Joint(4),pd.Joint(9)

# You could also send all joints as a list to DxlComm varible
serial.attach_joints([vacum, gripper, cutter])


# Enable single joint torque or all joints torques
print("Enable torque")
#yn.enable_torque()
serial.enable_torques()
sleep(0.5)

angles = serial.get_angles()
print(angles)

# anglesb4 = dyn.get_angle()
# print(anglesb4)
cutter.send_angle

# for i in range(10):
#     inputAngle = 10*i + 60
#     angles = dyn.get_angle()
#     print(angles)
#     dyn.send_angle(inputAngle)
#     sleep(1)
    



