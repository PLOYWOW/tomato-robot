import pyrealsense2 as rs
import cv2
import cv2.aruco as aruco
import numpy as np
import math
# import os

def  cameraSetting(width=1280, height=720):
    # Get device product line for setting a supporting resolution
    pipeline = rs.pipeline()
    config = rs.config() # Configure streams

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    # device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

    cfg = pipeline.start(config)

    # Camera calibration data
    profile = cfg.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
    intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics

    camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float32) 
    camera_distortion = np.array([0,0,0,0,0], dtype=np.float32)

    return pipeline, camera_matrix, camera_distortion,intr

def cameraProcess(pipeline):
    align_to = rs.stream.color
    align = rs.align(align_to)

    frames = pipeline.wait_for_frames()
        
    # Align the depth frame to color frame
    aligned_frames = align.process(frames)
    
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if not depth_frame or not color_frame: pass

    hole_filling = rs.hole_filling_filter()
    filled_frame = hole_filling.process(depth_frame)
    
    # Convert images to numpy arrays
    filled_depth_image = np.asanyarray(filled_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    return filled_depth_image, color_image,filled_frame

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
    rotation_flip = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]], dtype=np.int8)
    bufferMatrix = np.array([[0,0,0,1]])

    ## marker position relative to camera frame
    rotation_cameratomarker = np.matrix(cv2.Rodrigues(rvec)[0])
    rotation_markertocamera = rotation_cameratomarker.T

    #-- Get the orientation in terms of euler 321 (Needs to be flipped first)
    rotationMatrix_marker = rotation_flip*rotation_markertocamera
    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(rotationMatrix_marker)

    xyz_marker = np.array(tvec, dtype=np.float32)/1000
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

    return ht_marker#marker_pos, camera_pos, ht_marker

# finding all the aruco markers in the image while drawing a bounding box around the markers and return bounding box and id
def findArucoMarkers(img, arucoDict, arucoParam, camera_matrix, camera_distortion, draw=True):
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
    boxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam, 
                                              cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    
    if draw:
        aruco.drawDetectedMarkers(img, boxs)

    return [boxs, ids]

def getColorBound():
    
    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")

    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")
    
    #l_b = np.array([l_h, l_s, l_v])
    #u_b = np.array([u_h, u_s, u_v])
    
    l_b = np.array([49,100,33])
    u_b = np.array([147,255,255])
    return l_b,u_b

def getBlueDoctorPixel(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    l_b,u_b = getColorBound()
    mask = cv2.inRange(hsv, l_b, u_b)
    cX,cY = 0,0
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    xB,yB,w,h = cv2.boundingRect(c)
    blueDoctorDict = {"TopLeftX" : xB ,"TopLeftY" : yB,"Width" : w,"Height" : h,"CenterX" : cX,"CenterY" : cY,}
    return blueDoctorDict

def get_depth_at_pixel(depth_frame, pixel_x, pixel_y):
    return depth_frame.as_depth_frame().get_distance(round(pixel_x), round(pixel_y))
def convert_depth_pixel_to_metric_coordinate(depth, pixel_x, pixel_y, camera_intrinsics):
    print(camera_intrinsics.fx)
    X = (pixel_x - camera_intrinsics.ppx)/camera_intrinsics.fx *depth
    Y = (pixel_y - camera_intrinsics.ppy)/camera_intrinsics.fy *depth
    return np.array([X,Y,depth,1])


def main():
    # Define marker
    id_to_find  = 72
    marker_size  = 100 # [mm]

    width=1280 ; height=720

    # Camera calibration data
    pipeline, camera_matrix, camera_distortion,intr = cameraSetting(width, height)
    
    arucoDict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    arucoParam = aruco.DetectorParameters_create()

    # cap = cv2.VideoCapture(1) # or 0
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # print(f'----------------------------{cap}----------------------------')


    while True:
        depth_image, color_image,depth_frame = cameraProcess(pipeline)
        
        #cv2.imshow("flip", fliper)
        # filled_depth, depth_image, img = pipeline.wait_for_frames()
        # print(img)
        # success, img = cap.read()
        arucoFound = findArucoMarkers(color_image, arucoDict, arucoParam, camera_matrix, camera_distortion)

        Camera_to_Marker_Matrix = np.asmatrix(np.identity(4))
        print(type(Camera_to_Marker_Matrix))

        # loop through all the markets and augment each one
        if len(arucoFound[0]) != 0:
            for box, id in zip(arucoFound[0], arucoFound[1]):
                if int(id) == id_to_find:
                    Camera_to_Marker_Matrix = wantMarker(color_image, box, marker_size, camera_matrix, camera_distortion)

        # Camera axis
        #print(Camera_to_Marker_Matrix)
        Marker_to_Camera_Matrix = np.linalg.inv(Camera_to_Marker_Matrix) 
        print(Marker_to_Camera_Matrix)
        center_point = (int(width/2), int(height/2))
        cv2.line(color_image,center_point,(center_point[0]+10,center_point[1]),(0,0,255),1)
        cv2.line(color_image,center_point,(center_point[0],center_point[1]+10),(0,255,0),1)

        
        blueDoctorPixelDict = getBlueDoctorPixel(color_image)
        xB,yB,w,h,cX,cY = blueDoctorPixelDict["TopLeftX"],blueDoctorPixelDict["TopLeftY"],blueDoctorPixelDict["Width"],blueDoctorPixelDict["Height"],blueDoctorPixelDict["CenterX"],blueDoctorPixelDict["CenterY"]
        cv2.rectangle(color_image,(xB,yB),(xB+w,yB+h),(255,255,0),2)

        #get distance##
        dap = get_depth_at_pixel(depth_frame, cX, cY)
        print(dap)
        position_cam = convert_depth_pixel_to_metric_coordinate(dap, cX, cY, intr)
        position_wall = np.dot(Marker_to_Camera_Matrix,position_cam)
        print(f'blueDoctor pixel is at {position_wall}')
        
        xWall = position_wall.item((0,0))
        yWall = position_wall.item((0,1))
        zWall = position_wall.item((0,2))
        #cv2.putText(color_image,f"({position_cam[0]:.3f},{position_cam[1]:.3f},{position_cam[2]:.3f})", (xB,yB), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
        cv2.putText(color_image,f"({xWall:.3f},{yWall:.3f},{zWall:.3f})", (xB,yB), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        depth_dim = depth_colormap.shape
        color_dim = color_image.shape
        #print(depth_dim,color_dim)
        # if depth_dim != color_dim:
        resized_depth_image = cv2.resize(depth_colormap, dsize = (640, 480), interpolation=cv2.INTER_AREA)
        resized_color_image = cv2.resize(color_image, dsize= (640, 480), interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, resized_depth_image))

        #cv2.imshow("both", images)
        cv2.imshow("image", color_image)
        cv2.waitKey(1)


    # pipeline.stop()
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