#!/usr/bin/env python3
import pyrealsense2 as rs
import cv2
import cv2.aruco as aruco
import numpy as np
import math
# import os

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def poseCalculate(rvec, tvec):
    rotation_flip = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]], dtype=np.int8)
    bufferMatrix = np.array([[0,0,0,1]])

    ## marker position relative to camera frame
    rotation_cameratomarker = np.matrix(cv2.Rodrigues(rvec)[0])
    rotation_markertocamera = rotation_cameratomarker.T

    #-- Get the orientation in terms of euler 321 (Needs to be flipped first)
    rotationMatrix_marker = rotation_flip*rotation_markertocamera
    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(rotationMatrix_marker)

    xyz_marker = np.array(tvec, dtype=np.float32)
    rpy_marker = np.array([math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker)], dtype=np.float32)
    marker_pos = np.concatenate((xyz_marker, rpy_marker), axis=0)
    ht_marker = np.concatenate((rotationMatrix_marker,np.array([xyz_marker]).T ) , axis=1)
    ht_marker = np.concatenate((ht_marker , bufferMatrix), axis=0)

    ## camera position relative to marker
    xyz_camera = np.array(-rotation_markertocamera*np.matrix(tvec).T, dtype=np.float32)
    xyz_camera = xyz_camera.reshape(3,)
    roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(rotation_flip*rotation_markertocamera)
    rpy_camera = np.array([math.degrees(roll_camera), math.degrees(pitch_camera), math.degrees(yaw_camera)], dtype=np.float32)
    camera_pos = np.concatenate((xyz_camera, rpy_camera), axis=0)

    return marker_pos, camera_pos, ht_marker

def wantMarker(img, box, marker_size, camera_matrix, camera_distortion, drawID=True):
    ret = aruco.estimatePoseSingleMarkers(box, marker_size, camera_matrix, camera_distortion)
    #-- Unpack the output, get only the first
    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] # rotation and translation vector

    # aruco.drawDetectedMarkers(img, boxs)
    aruco.drawAxis(img, camera_matrix, camera_distortion, rvec, tvec, 10)
    marker_pos, camera_pos, ht_marker = poseCalculate(rvec, tvec)

    if drawID:
        marker_pos = "Marker position x=%4.4f y=%4.4f z=%4.4f roll=%4.4f pitch=%4.4f yaw=%4.4f"%(marker_pos[0], marker_pos[1], marker_pos[2], marker_pos[3], marker_pos[4], marker_pos[5])
        cv2.putText(img, marker_pos, (10, 50), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,255), 2)

    return

# finding all the aruco markers in the image while drawing a bounding box around the markers and return bounding box and id
def findArucoMarkers(img, arucoDict, arucoParam, camera_matrix, camera_distortion, draw=True):
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
    boxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam, 
                                              cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    
    if draw:
        aruco.drawDetectedMarkers(img, boxs)

    return [boxs, ids]

def main():

    # Define marker
    id_to_find  = 72
    marker_size  = 100 # [mm]

    # Camera calibration data
    pipe = rs.pipeline() # Create a context object. This object owns the handles to all connected realsense devices

    width, height = 1280, 720
    config = rs.config() # Configure streams
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)

    profile = pipe.start(config)
    profile = profile.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
    intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
    camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float32) 
    camera_distortion = np.array([0,0,0,0,0], dtype=np.float32)
    # calib_path = "camera_calibation"
    # camera_matrix = np.loadtxt(os.path.join(calib_path,'cameraMatrix.txt'), delimiter=',')
    # camera_distortion = np.loadtxt(os.path.join(calib_path,'cameraDistortion.txt'), delimiter=',')

    arucoDict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    arucoParam = aruco.DetectorParameters_create()

    cap = cv2.VideoCapture(0) # or 0
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    print(f'----------------------------{cap}----------------------------')


    while True:
        success, img = cap.read()
        print(success)
        # return
        arucoFound = findArucoMarkers(img, arucoDict, arucoParam, camera_matrix, camera_distortion)

        # loop through all the markets and augment each one
        if len(arucoFound[0]) != 0:
            for box, id in zip(arucoFound[0], arucoFound[1]):
                if int(id) == id_to_find:
                    wantMarker(img, box, marker_size, camera_matrix, camera_distortion)

        # Camera axis
        center_point = (int(width/2), int(height/2))
        cv2.line(img,center_point,(center_point[0]+100,center_point[1]),(0,0,255),1)
        cv2.line(img,center_point,(center_point[0],center_point[1]+100),(0,255,0),1)

        cv2.imshow("image", img)
        cv2.waitKey(1)


if __name__ == '__main__':
    main()

"""
marker axis:
                A y
                |
                |
                |tag center
                O---------> x
camera axis:
                X--------> x
                | frame center
                |
                |
                V y
"""