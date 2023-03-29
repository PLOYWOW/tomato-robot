import cv2 as cv
import numpy as np


maxScaleUp = 100
scaleFactor = 50
windowName = "Resize Image"
trackbarValue = "Scale"

#Create image
image = np.zeros((10,300,3), np.uint8)

#Create a window to display results and set the flag to Autosize
cv.namedWindow(windowName, cv.WINDOW_AUTOSIZE)

#Call back function
def scaleImage(*args):
    scaleFactor = 1+args[0]/100.0
    scaledImage = cv.resize(image,None,fx=scaleFactor,fy=scaleFactor,interpolation=cv.INTER_LINEAR)
    # cv.imshow(windowName, scaledImage)

#Create trackbar
cv.createTrackbar(trackbarValue,windowName,scaleFactor,maxScaleUp,scaleImage)

#Display the image
# cv.imshow(windowName,image)
c = cv.waitKey(0)
cv.destroyAllWindows()