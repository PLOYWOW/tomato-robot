import numpy as np
import cv2

cap = cv2.VideoCapture(2)

def nothing(x):
    pass

cv2.namedWindow('Mode')
cv2.createTrackbar('R','Mode',0,255,nothing)
cv2.createTrackbar('G','Mode',0,255,nothing)
cv2.createTrackbar('B','Mode',0,255,nothing)

cv2.createTrackbar('R_L','Mode',0,255,nothing)
cv2.createTrackbar('G_L','Mode',0,255,nothing)
cv2.createTrackbar('B_L','Mode',0,255,nothing)

while True :
    
    ref,frame = cap.read()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    r = cv2.getTrackbarPos('R','Mode')
    g = cv2.getTrackbarPos('G','Mode')
    b = cv2.getTrackbarPos('B','Mode')

    r_l = cv2.getTrackbarPos('R_L','Mode')
    g_l = cv2.getTrackbarPos('G_L','Mode')
    b_l = cv2.getTrackbarPos('B_L','Mode')

    lower = np.array([b_l,g_l,r_l])
    upper = np.array([b,g,r])

    mask = cv2.inRange(hsv, lower, upper)

    res = cv2.bitwise_and(frame,frame, mask= mask)

   
    cv2.imshow("Mode",res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
