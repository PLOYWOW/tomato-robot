import cv2
import numpy as np

img_path = "./test_watershade/tomato.jpg"

img = cv2.imread(img_path)

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)


cv2.imshow("gray bf erode",gray)

kernel = np.ones((13,13),np.uint8)
gray_erode = cv2.erode(gray,kernel,iterations=1)

img_gray_his = cv2.equalizeHist(gray_erode)

img_gray_his = cv2.blur(img_gray_his,(7,7))
img_edge_gray = cv2.Canny(img_gray_his,0,150)

cv2.imshow("img_edge_gray",img_edge_gray)

# gray_erode[np.where(img_edge_gray >=1)] = 0

#extracted_image = cv2.bitwise_and(img_edge_hsv, img_gray_bin)

S = hsv[:,:,1]
S = np.array(S,dtype=np.uint8)
S[np.where(gray_erode ==0)] = 0

circles = cv2.HoughCircles(S, cv2.HOUGH_GRADIENT, 1, 10, param1=100, param2=60, minRadius=95, maxRadius=0)
print(circles)

circles = np.uint16(np.around(circles))



for c in circles[0, :]:
    cv2.circle(img, (c[0], c[1]), c[2], (0, 255, 0), 3)
    cv2.circle(img, (c[0], c[1]), 1, (0, 0, 255), 5)

cv2.imshow("img",img)
cv2.imshow("S",S)

ret, bin_img = cv2.threshold(S, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

# ret, bin_img = cv2.threshold(gray_erode, 1, 255, cv2.THRESH_BINARY)
# ret, bin_img_noerode = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

dist = cv2.distanceTransform(bin_img,cv2.DIST_L2,5)
cv2.imshow("dist",dist)

# fig, ax = plt.subplots(figsize=(6, 6))
# ax.imshow(dist, cmap="magma")
# plt.show()

# redmask = cv2.dilate(redmask, kernel, iterations = 1)

# 距離を利用して2値化する。
ret, bin_img2 = cv2.threshold(dist, 0.2 * dist.max(), 255, cv2.THRESH_BINARY)
# imshow(bin_img2)



# ret, bin_img = cv2.threshold(gray,1,255,cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

# cv2.imshow("gray after erode",gray)
cv2.imshow("bin",bin_img)
# cv2.imshow("bin no erode",bin_img_noerode)
cv2.imshow("dist",bin_img2)

cv2.waitKey(0)
cv2.destroyAllWindows()