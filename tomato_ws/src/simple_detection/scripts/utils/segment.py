import cv2
import numpy as np

class segment():
    def __init__(self,redmask):
        self.img = redmask

    def instance_segment(self):
        # Define the path to your resources
        path_to_model = './utils/frozen_inference_graph_coco.pb'
        path_to_config = './utils/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt'
        
        first = np.array([[0,0,255],[255,0,0],[0,255,0],[0,255,255],[255,0,255],[255,255,0]])
        colors_random = np.random.randint(125, 255, (80, 3))
        colors = np.vstack((first,colors_random))

        # Define variables and parameters
        model = cv2.dnn.readNetFromTensorflow(path_to_model,path_to_config)
        height, width, _ = self.img.shape
        black_image = np.zeros((height, width, 3), np.uint8)
        black_image[:] = (0, 0, 0)

        # Preprocess and Detecting objects
        blob = cv2.dnn.blobFromImage(self.img, swapRB=True)
        model.setInput(blob)
        boxes, masks = model.forward(["detection_out_final", "detection_masks"])
        no_of_objects = boxes.shape[2]

        # Fetching Object Wise Outputs
        for i in range(no_of_objects):
            box = boxes[0, 0, i]
            class_id = box[1]
            score = box[2]
            if score < 0.5:
                continue
            # Getting the coordinates of our objects
            x = int(box[3] * width)
            y = int(box[4] * height)
            x2 = int(box[5] * width)
            y2 = int(box[6] * height)
            # Building our Mask
            roi = black_image[y: y2, x: x2]
            roi_height, roi_width, _ = roi.shape
            mask = masks[i, int(class_id)]
            mask = cv2.resize(mask, (roi_width, roi_height))
            _, mask = cv2.threshold(mask, 0.5, 255, cv2.THRESH_BINARY)
            cv2.rectangle(self.img, (x, y), (x2, y2), (0,255,255), 3)
            # Finding the contours and drawing them
            contours, _ = cv2.findContours(np.array(mask, np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # color = colors[int(class_id)]
            color = colors[i]
            for cnt in contours:
                cv2.fillPoly(roi, [cnt], (int(color[0]), int(color[1]), int(color[2])))
        # Visualizing the results
        # cv2.imshow("Final",np.hstack([img,black_image]))
        # cv2.imshow("black_image",black_image)
        # cv2.imshow("img",img)
        overlay_frame = ((1*black_image)+(0*self.img)).astype("uint8")
        transparent_overlay_frame = ((0.6*black_image)+(0.4*self.img)).astype("uint8")
        return (overlay_frame,transparent_overlay_frame)