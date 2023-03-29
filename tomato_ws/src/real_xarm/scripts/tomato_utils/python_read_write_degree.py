#!/usr/bin/env python


import PyDynamixel_v2 as pd
from math import pi
import time

# Declare the DxlComm variable
port = "/dev/ttyUSB0"
baudrate = 57600
serial = pd.DxlComm(port=port, baudrate=baudrate)

# Declare a dynamixel joint
vacum,gripper = pd.Joint(4,max_value=1024,max_angle=300),pd.Joint(2,max_value=2048,max_angle=180)

# You could also send all joints as a list to DxlComm varible
serial.attach_joints([vacum, gripper])


# Enable single joint torque or all joints torques
print("Enable torque")
#yn.enable_torque()
serial.enable_torques()
time.sleep(0.5)

angles = serial.get_angles()
print(angles)

# anglesb4 = dyn.get_angle()
# print(anglesb4)

result = gripper.send_profile_velocity(value=100)
print(result)
vacum.send_angle(90)
time.sleep(1)
gripper.send_angle(163) #221
time.sleep(1)
#cutter.send_angle(235)#235 #144120
#serial.send_angles([90,130])

# for i in range(10):
#     inputAngle = 10*i + 60
#     angles = dyn.get_angle()ss
#     print(angles)
#     dyn.send_angle(inputAngle)
#     sleep(1)
    



