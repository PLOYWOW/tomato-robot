import cv2

# image1 = cv2.imread("/home/hayashi/Downloads/watershed_coins_01.jpg")
# w = Watershed()
# image2 = w.apply(image1)

from Watershed import *

img = cv2.imread("tomato.jpg")
img_resize = cv2.resize(img,(int(1280/4),int(720/4)))
# img_resize = cv2.resize(img,(int(1280),int(720)))
cv2.imwrite("tomato_resize.jpg",img_resize)

shed = Watershed(
            data_image = "tomato_resize.jpg",
            binary_or_gray_or_color = "color",
            size_for_calculations = 128,
            sigma = 1,
            gradient_threshold_as_fraction = 0.1,
            level_decimation_factor = 16,
            padding = 5,
        )
shed.extract_data_pixels()
shed.display_data_image()
# shed.mark_image_regions_for_gradient_mods()                     #(A)
shed.compute_gradient_image()
# shed.modify_gradients_with_marker_minima()                      #(B)
shed.compute_Z_level_sets_for_gradient_image()
shed.propagate_influence_zones_from_bottom_to_top_of_Z_levels()
shed.display_watershed()
shed.display_watershed_in_color()
shed.extract_watershed_contours_separated()
shed.display_watershed_contours_in_color()



print("OKAY")

# cv2.imshow("Original image",image1)
# cv2.waitKey(0)
# cv2.destroyAllWindows()