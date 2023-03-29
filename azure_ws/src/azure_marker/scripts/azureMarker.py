#!/usr/bin/env python


import numpy as np
import os
import math
import time
import cv2

import struct
from cv_bridge import CvBridge
from tf2_ros.buffer_interface import TransformRegistration
bridge = CvBridge()

import rospy
import ros_numpy
from std_msgs.msg import Header,String
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField, Image
from std_msgs.msg import Bool,Float32MultiArray
from visualization_msgs.msg import Marker,MarkerArray
import tf

import glob
from geometry_msgs.msg import Point,Pose,PoseStamped,PoseArray,PointStamped
import ctypes

import open3d as o3d

import matplotlib.pyplot as plt

import cv2.aruco as aruco



class DepthImageHandler(object):
    def __init__(self):
        self.CAMINFO = {'topic': '/depth_to_rgb/camera_info', 'msg': CameraInfo}
        self.isCamInfo = False

        self.COLOR = {'topic': '/rgb/image_raw', 'msg': Image}
        self.DEPTH = {'topic': '/depth_to_rgb/image_raw', 'msg': Image}
        self.PC = {'topic': '/camera/depth/color/points', 'msg': PointCloud2}
        
        self.H = 720
        self.W = 1280
        self.cropSize = 4
        self.header = Header()
        self.fields = [PointField('x', 0, 7, 1), PointField('y', 4, 7, 1), PointField('z', 8, 7, 1), PointField('rgb', 16, 7, 1)]
        self.points = []
        self.pc = PointCloud2()

        self.color_image = np.empty((self.H, self.W ,3), dtype=np.uint8)
        self.depth_image = np.empty((self.H, self.W), dtype=np.uint16)
        self.aligned_image  = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask           = np.empty((self.H, self.W), dtype=np.bool)
        self.mask_image     = np.empty((self.H, self.W), dtype=np.uint8)
        self.mask_depth     = np.empty((self.H, self.W), dtype=np.uint8)
        self.cropPointCloud = o3d.geometry.PointCloud() 

        self.camera_matrix = np.array([[0.0, 0, 0.0], [0, 0.0, 0.0], [0, 0, 1]], dtype=np.float32) 
        self.camera_distortion = np.array([0,0,0,0,0], dtype=np.float32)
        



    def camInfoCallback(self, msg):
        self.header = msg.header
        self.K = msg.K
        self.width = msg.width  
        self.height = msg.height
        self.ppx = msg.K[2]
        self.ppy = msg.K[5]
        self.fx = msg.K[0]
        self.fy = msg.K[4] 
        
        self.model = msg.distortion_model
        self.k1 = msg.D[0]
        self.k2 = msg.D[1]
        self.t1 = msg.D[2]
        self.t2 = msg.D[3]
        self.k3 = msg.D[4]
        self.isCamInfo = True



        self.camera_matrix = np.array([[self.fx, 0, self.ppx], [0, self.fy, self.ppy], [0, 0, 1]], dtype=np.float32) 
        self.camera_distortion = np.array([self.k1,self.k2,self.t1,self.t2,self.k3], dtype=np.float32)

        


    def colorCallback(self, msg):
        self.color_image = ros_numpy.numpify(msg)
        #self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)

    def depthCallback(self, msg):
        self.depth_image = ros_numpy.numpify(msg).astype(np.uint16)
    
    def find_nearest_nonzero(self,depth_img, target):
        #nonzero = cv2.findNonZero(depth_img)
        #distances = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 + (nonzero[:,:,1] - target[1]) ** 2)
        #nearest_index = np.argmin(distances)
        zeroPixel = depth_img[target[1],target[0]]
        count = 0
        while zeroPixel == 0:
            count += 1
            zeroPixel = depth_img[target[1],target[0]+count]
            if count > 5:
                break

        return target[0]+count,target[1]

    def depthToPointsDistance(self, depth_image, U, V):
        V =  np.clip(V,0,self.height-1)
        U =  np.clip(U,0,self.width-1)  
        
        x = (U - self.K[2])/self.K[0]
        y = (V - self.K[5])/self.K[4]      
        U,V = self.find_nearest_nonzero(depth_image,(U,V))
        #print(vv,uu)
        z = depth_image[V,U] / 1000.0
        x *= z
        y *= z
        # print(U,V,x,y,z)
        point3 = [x,y,z]
        return point3
        # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    def rotationMatrixToEulerAngles(self,R):
        assert (self.isRotationMatrix(R))

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

    def poseCalculate(self,rvec, tvec):
        rotation_flip = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]], dtype=np.int8)
        bufferMatrix = np.array([[0,0,0,1]])

        ## marker position relative to camera frame
        rotation_cameratomarker = np.matrix(cv2.Rodrigues(rvec)[0])
        rotation_markertocamera = rotation_cameratomarker.T

        #-- Get the orientation in terms of euler 321 (Needs to be flipped first)
        # rotationMatrix_marker = rotation_flip*rotation_markertocamera
        # roll_marker, pitch_marker, yaw_marker = self.rotationMatrixToEulerAngles(rotationMatrix_marker)

        rotationMatrix_camera = rotation_flip*rotation_cameratomarker
        roll_marker, pitch_marker, yaw_marker = self.rotationMatrixToEulerAngles(rotationMatrix_camera)

    
        xyz_camera = np.array(tvec, dtype=np.float32)/1000
        rpy_camera = np.array([math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker)], dtype=np.float32)
        marker_pos = np.concatenate((xyz_camera, rpy_camera), axis=0)
        #ht_marker = np.concatenate((rotationMatrix_camera,np.array([xyz_marker]).T ) , axis=1)
        #ht_marker = np.concatenate((ht_marker , bufferMatrix), axis=0)

        ## camera position relative to marker
        xyz_camera = np.array(-rotation_markertocamera*np.matrix(tvec).T, dtype=np.float32)/1000
        xyz_camera = xyz_camera.reshape(3,)
        roll_camera, pitch_camera, yaw_camera = self.rotationMatrixToEulerAngles(rotation_flip*rotation_markertocamera)
        rpy_camera = np.array([math.degrees(roll_camera), math.degrees(pitch_camera), math.degrees(yaw_camera)], dtype=np.float32)
        camera_pos = np.concatenate((xyz_camera, rpy_camera), axis=0)

        return marker_pos,camera_pos#, ht_marker

    def getMarkerPose(self,img, box, marker_size, camera_matrix, camera_distortion, drawID=True):
        ret = aruco.estimatePoseSingleMarkers(box, marker_size, camera_matrix, camera_distortion)
        #-- Unpack the output, get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] # rotation and translation vector

        # aruco.drawDetectedMarkers(img, boxs)
        aruco.drawAxis(img, camera_matrix, camera_distortion, rvec, tvec, 10)
        #marker_pos, camera_pos, ht_marker = self.poseCalculate(rvec, tvec)
        marker_pos,camera_pos = self.poseCalculate(rvec, tvec)

        if drawID:
            marker_pos_text = "Marker position x=%4.4f y=%4.4f z=%4.4f roll=%4.4f pitch=%4.4f yaw=%4.4f"%(marker_pos[0], marker_pos[1], marker_pos[2], marker_pos[3], marker_pos[4], marker_pos[5])
            cv2.putText(img, marker_pos_text, (10, 50), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,255), 2)

        return marker_pos#ht_marker#marker_pos, camera_pos, ht_marker
    
    def getCameraPose(self,img, box, marker_size, camera_matrix, camera_distortion, drawID=True):
        ret = aruco.estimatePoseSingleMarkers(box, marker_size, camera_matrix, camera_distortion)
        #-- Unpack the output, get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] # rotation and translation vector

        # aruco.drawDetectedMarkers(img, boxs)
        aruco.drawAxis(img, camera_matrix, camera_distortion, rvec, tvec, 10)
        #marker_pos, camera_pos, ht_marker = self.poseCalculate(rvec, tvec)
        marker_pos,camera_pos = self.poseCalculate(rvec, tvec)

        if drawID:
            camera_pos_text = "Marker position x=%4.4f y=%4.4f z=%4.4f roll=%4.4f pitch=%4.4f yaw=%4.4f"%(camera_pos[0], camera_pos[1], camera_pos[2], camera_pos[3], camera_pos[4], camera_pos[5])
            cv2.putText(img, camera_pos_text, (10, 50), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,255), 2)

        return camera_pos#ht_marker#marker_pos, camera_pos, ht_marker
    

    # finding all the aruco markers in the image while drawing a bounding box around the markers and return bounding box and id
    def findArucoMarkers(self,img, arucoDict, arucoParam, camera_matrix, camera_distortion, draw=True):
        imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
        boxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam, 
                                                cameraMatrix=camera_matrix, distCoeff=camera_distortion)
        
        if draw:
            aruco.drawDetectedMarkers(img, boxs)

        return [boxs, ids]  

    def boardCastInverseTF(self):
        br = tf.TransformBroadcaster()
        try:
            rgb2depth = self.camTF.lookupTransform("rgb_camera_link", "depth_camera_link", rospy.Time(0))
            #print(f"rgb2dpeth{rgb2depth}")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        try:
            depth2base = self.camTF.lookupTransform("depth_camera_link", "camera_base", rospy.Time(0))
            #print(f"depth2rgb{depth2base}")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        br.sendTransform((rgb2depth[0][0], rgb2depth[0][1],rgb2depth[0][2]),
                    (rgb2depth[1][0],rgb2depth[1][1],rgb2depth[1][2],rgb2depth[1][3]),
                     rospy.Time.now(),
                     "depth",
                     "rgb")
        br.sendTransform((depth2base[0][0], depth2base[0][1],depth2base[0][2]),
                    (depth2base[1][0],depth2base[1][1],depth2base[1][2],depth2base[1][3]),
                     rospy.Time.now(),
                     "base",
                     "depth")


    def boardCastMarkerFromCameraTF(self,markerPose):
        br = tf.TransformBroadcaster()
        pos_list,rot_list = markerPose[0:3],markerPose[3:6]
    
        rot_rad_list = [math.radians(float(angle)) for angle in rot_list]
        br.sendTransform((pos_list[0], pos_list[1],pos_list[2]),
                     tf.transformations.quaternion_from_euler(rot_rad_list[0], rot_rad_list[1], rot_rad_list[2]),
                     rospy.Time.now(),
                     "rgb",
                     "az_marker")
    
    def getTF(self,targetFrame = "rgb_camera",baseFrame = "link_base"):
        try:
            (trans,rot) = self.camTF.lookupTransform(baseFrame, targetFrame, rospy.Time(0))
            rotDegree = tf.transformations.euler_from_quaternion(rot)
            return (trans,rotDegree)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return "NO TF"
                     

    def findRotationFromTo(self,VecA,VecB):
        v = np.cross(VecA,VecB)
        s = np.linalg.norm(v)
        c = np.dot(VecA,VecB)
        I = np.identity(3)
        vXStr = '{} {} {}; {} {} {}; {} {} {}'.format(0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0)
        k = np.matrix(vXStr)
        r = I + k + np.matmul(k,k) * ((1 -c)/(s**2))
        return r

    def publishPoint3(self,pos3):
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/rgb_camera_link"


        point.point.x = pos3[0]
        point.point.y = pos3[1]
        point.point.z = pos3[2]
        
        self.point_pub.publish(point)
    
    
    
    
        
        
    
    def GetMarker(self):

        color_image, depth_image    = self.color_image, np.asanyarray(self.depth_image)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
        #print(color_image.shape)
        depthForCloud = depth_image
        # ================================================================================ CHANGED ============
        # converted to 16bit
        depth_image = depth_image.astype(np.uint16)

        markerID  = 50
        marker_size  = 150
        arucoDict = aruco.getPredefinedDictionary(aruco.DICT_7X7_100)
        arucoParam = aruco.DetectorParameters_create()
        arucoFound = self.findArucoMarkers(color_image, arucoDict, arucoParam, self.camera_matrix, self.camera_distortion)

        #Camera_to_Marker_Matrix = np.asmatrix(np.identity(4))
        #print(type(Camera_to_Marker_Matrix))

        # loop through all the markets and augment each one
        if len(arucoFound[0]) != 0:
            for box, id in zip(arucoFound[0], arucoFound[1]):
                if int(id) == markerID:
                    # (topLeft, topRight, bottomRight, bottomLeft) = box[0][0],box[0][1],box[0][2],box[0][3]
                    # topLeft = [int(x) for x in topLeft]
                    # topRight = [int(x) for x in topRight]
                    # bottomRight = [int(x) for x in bottomRight]
                    # bottomLeft = [int(x) for x in bottomLeft]

                    # cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    # cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    # cv2.circle(color_image, (cX, cY), 4, (0, 0, 255), -1)
                    # tX = int((topLeft[0] + topRight[0]) / 2.0)
                    # tY = int((topLeft[1] + topRight[1]) / 2.0)
                    # cv2.circle(color_image, (tX, tY), 4, (0, 0, 255), -1)
                    # lX = int((topLeft[0] + bottomLeft[0]) / 2.0)
                    # lY = int((topLeft[1] + bottomLeft[1]) / 2.0)
                    # cv2.circle(color_image, (lX, lY), 4, (0, 0, 255), -1)



                    # centerPoint3 = self.depthToPointsDistance(depth_image,cX,cY)
                    # topPoint3 = self.depthToPointsDistance(depth_image,tX,tY)
                    # #topRight3 = self.depthToPointsDistance(depth_image,topRight[0],topRight[1])
                    # leftPoint3 = self.depthToPointsDistance(depth_image,lX,lY)

                    # #self.publishPoint3(topPoint3)
                    # self.publishPoint3(leftPoint3)


                    # centerArray = np.array(centerPoint3)
                    # topArray= np.array(topPoint3)
                    # leftArray = np.array(leftPoint3)
                    
                    # upArray = np.cross(topArray,leftArray)
                    # normUpArray = upArray/np.linalg.norm(upArray)
                    

                    # #normCenter2topArray = (topPoint3 - centerArray)/np.linalg.norm(topPoint3 - centerArray)
                    # axisArray = np.array([1.0,0.0,0.0])
                    # normAxisArray = axisArray/np.linalg.norm(axisArray)
                    # #print(normfrontArray)
                    
                    # rotationCamera_Marker = self.findRotationFromTo(normUpArray,normAxisArray)
                    # rotationMarker_Camera = np.linalg.inv(rotationCamera_Marker)
                    
                    # #Checker
                    # center2top = np.dot(rotationCamera_Marker, (topArray - centerArray))
                    #print(center2top)
                    
                    #RPY = self.rotationMatrixToEulerAngles(rotationCamera_Marker)
                    #print(math.degrees(RPY[0]),math.degrees(RPY[1]),math.degrees(RPY[2]))



                    markerPoseFromCam = self.getCameraPose(color_image, box, marker_size, self.camera_matrix, self.camera_distortion)
                    #markerPoseFromCam[0:3] = centerPoint3
                    #markerPoseFromCam[3:6] = RPY
                    self.boardCastInverseTF()
                    self.boardCastMarkerFromCameraTF(markerPoseFromCam)
                    #fromCamBaseTF = self.getTF(camFrame = "az_marker", baseFrame = "rgb_camera_link")
                    fromWorldTF = self.getTF(targetFrame = "base", baseFrame = "link_base")
                    #print(f"cam2rgb->{fromCamBaseTF}")
                    print(f"base2cam->{fromWorldTF}")
        cv2.imshow("image", color_image)
        cv2.waitKey(1)




    def process(self):
        rospy.init_node('markerFinder', anonymous=True)
        #r = rospy.Rate(60)
        rospy.Subscriber(self.CAMINFO['topic'], self.CAMINFO['msg'], self.camInfoCallback)
        rospy.Subscriber(self.COLOR['topic'], self.COLOR['msg'], self.colorCallback)
        rospy.Subscriber(self.DEPTH['topic'], self.DEPTH['msg'], self.depthCallback)
        #rospy.Subscriber(self.PC['topic'],      self.PC['msg'],         self.pcCallback)
        
        #self.depth_scale    = 0.0010000000474974513

        ###publisher
        self.markerXYZRPY_pub = rospy.Publisher("markerXYZRPY", Float32MultiArray, queue_size=50)

        self.point_pub = rospy.Publisher('/goal_point', PointStamped,
                                          queue_size=1)
        self.camTF = tf.TransformListener()


        while not rospy.is_shutdown():
                self.GetMarker()
         
if __name__ == '__main__':
    try:
        _depthImageHandler = DepthImageHandler()
        _depthImageHandler.process()

    except rospy.ROSInterruptException:
        pass
