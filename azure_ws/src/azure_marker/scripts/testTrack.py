import numpy as np
import cv2

window_name = "test"

def draw_circle(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),100,(255,0,0),-1)

img = np.zeros((512,512,3), np.uint8)

window_open = False
# just try as long as a matching window is found
while not window_open:
    try:
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, draw_circle)
        l_h = cv2.getTrackbarPos("LH", "Tracking")
        l_s = cv2.getTrackbarPos("LS", "Tracking")
        l_v = cv2.getTrackbarPos("LV", "Tracking")

        u_h = cv2.getTrackbarPos("UH", "Tracking")
        u_s = cv2.getTrackbarPos("US", "Tracking")
        u_v = cv2.getTrackbarPos("UV", "Tracking")
        window_open = True
    except:
        # destroy any "erroneous" windows OpenCV might have created
        cv2.destroyAllWindows()
    	
while(1):
    cv2.imshow(window_name, img)
    if cv2.waitKey(20) & 0xFF == 27:
        break
        
cv2.destroyAllWindows()
