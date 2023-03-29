import cv2
from utils.segment import segment

tomato = cv2.imread("./test_watershade/tomato.jpg")

# cv2.imshow("re",redmask)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

segment_ini = segment(tomato)
segmented_img = segment_ini.instance_segment()

print(segmented_img.shape)
cv2.imshow("Output", segmented_img)
cv2.waitKey(0)
cv2.destroyAllWindows()