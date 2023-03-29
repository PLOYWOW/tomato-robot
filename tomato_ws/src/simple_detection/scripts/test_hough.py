import cv2
import numpy as np

img_path = "./test_watershade/tomato.jpg"
img = cv2.imread(img_path)
height = img.shape[0]
width = img.shape[1]
print(img.shape)

img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
kernel = np.ones((5,5),np.uint8)
img_gray = cv2.erode(img_gray,kernel,iterations=1)

# cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# img_blur = cv2.GaussianBlur(img, (7, 7), 1.5)
# print(img_blur.shape)



# cv2.namedWindow("Blurred img",cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Blurred img",(int(width*2/3*2),int(height*2/3)))
# cv2.imshow("Blurred img",np.hstack([img,img_blur]))
# cv2.waitKey(0)
# cv2.destroyAllWindows()




circles = cv2.HoughCircles(img_gray, cv2.HOUGH_GRADIENT, 1, 200, param1=100, param2=30, minRadius=150, maxRadius=500)
print(circles)
print(type(circles))

circles = np.uint16(np.around(circles))

for c in circles[0, :]:
    cv2.circle(img, (c[0], c[1]), c[2], (0, 255, 0), 3)
    cv2.circle(img, (c[0], c[1]), 1, (0, 0, 255), 5)

cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
