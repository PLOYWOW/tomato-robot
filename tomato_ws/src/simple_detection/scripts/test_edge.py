import cv2
import numpy as np

img_path = "./test_watershade/tomato.jpg"
img = cv2.imread(img_path)
img_blur = cv2.blur(img,(1,1))
height = img.shape[0]
width = img.shape[1]

img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

kernel = np.ones((5,5),np.uint8)
img_gray = cv2.erode(img_gray,kernel,iterations=1)

ret, img_gray_bin = cv2.threshold(img_gray, 20, 255, cv2.THRESH_BINARY)

img_gray_his = cv2.equalizeHist(img_gray)

# img_edge = cv2.Canny(img_gray,0,120)
# img_edge = cv2.Canny(img_blur,0,120)
# img_edge = cv2.Canny(img_hsv,100,200)

img_gray_his = cv2.blur(img_gray_his,(5,5))

img_edge_gray = cv2.Canny(img_gray_his,0,100)

img_edge_hsv = cv2.Canny(img_hsv[:,:,2],40,200)

# img_gray_hsv_erode = img_edge_hsv[np.where(img_gray != 0)]
extracted_image = cv2.bitwise_and(img_edge_hsv, img_gray_bin)

sobelx = cv2.Sobel(src=img_gray_his,ddepth=cv2.CV_64F,dx=1,dy=0,ksize=11)

ret, threshold_img = cv2.threshold(img_gray_his, 60, 255, cv2.THRESH_BINARY)




# cv2.namedWindow("HSV",cv2.WINDOW_NORMAL)
# cv2.resizeWindow("HSV",(int(width*2/3*2),int(height*2/3)))
# cv2.imshow("HSV",np.hstack([img,img_hsv]))

# cv2.namedWindow("Blurred img",cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Blurred img",(int(height),int(width)))
# cv2.imshow("Blurred img",img_blur)
# cv2.resizeWindow("Blurred img",(int(width*2/3*2),int(height*2/3)))
# cv2.imshow("Blurred img",np.hstack([img,img_blur]))

# cv2.namedWindow("Edge img",cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Edge img",(int(width*2/3),int(height*2/3)))

# cv2.imshow("Sobelx",sobelx)

cv2.namedWindow("Edge img gray",cv2.WINDOW_NORMAL)
cv2.imshow("Edge img gray",img_edge_gray)

cv2.namedWindow("Edge img hsv",cv2.WINDOW_NORMAL)
cv2.imshow("Edge img hsv",img_edge_hsv)
# cv2.imshow("img_gray_hsv_erode",img_gray_hsv_erode)
# cv2.imshow("HSV",img_hsv[:,:,2])

cv2.namedWindow("GRAY with erode",cv2.WINDOW_NORMAL)
cv2.imshow("GRAY with erode",img_gray)
# cv2.imshow("Hist",img_gray_his)
# cv2.imshow("THRES",threshold_img)

cv2.namedWindow("extracted_image",cv2.WINDOW_NORMAL)
cv2.imshow("extracted_image",extracted_image)

cv2.waitKey(0)
cv2.destroyAllWindows()