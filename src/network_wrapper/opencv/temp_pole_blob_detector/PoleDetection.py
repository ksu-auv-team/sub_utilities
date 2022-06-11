#imports OpenCV and Numpy
import cv2
import numpy as np
import os

def findMaxHeight(b_list):
    temp_max_box = [0, 0, 0, 0]
    if len(b_list) == 1:
        # print("There was only one so" + str(b_list[0]))
        return b_list[0]
    for box in b_list:
        # print("temp_max box: " + str(temp_max_box) + " vs " + str(box))
        if box[3] > temp_max_box[3]:
            temp_max_box = box
        # print(str(temp_max_box) + " won")
    return temp_max_box

pole_mask_lower = np.array([0,0,0])
pole_mask_upper = np.array([50,40,40])

dir = "./pictures"
i = 0
for image_path in os.listdir(dir):
    print("run " + str(i))
    img = os.path.join(dir, image_path)
    frame = cv2.imread(img)

    pole_mask = cv2.inRange(frame, pole_mask_lower, pole_mask_upper)
    pole_mask = cv2.erode(pole_mask, None, iterations=2)
    pole_mask = cv2.dilate(pole_mask, None, iterations=2)

    # Get contours based on mask
    b_contours, b_hierarchy = cv2.findContours(pole_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # bitwise mask
    results = cv2.bitwise_and(frame, frame, mask=pole_mask)
    cv2.imwrite("./results/frame_with_basic_mask_%d.jpg" % i, results)
    
    # Binary mask 
    frame_grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(frame_grey, 20, 255, cv2.THRESH_BINARY_INV)
    cv2.imwrite("./results/frame_with_binary_mask_%d.jpg" % i, threshold)

    # Bounding boxes on those masks, really as long as 
    b_rects = []
    for c in b_contours:
        # print("run " + str(i) + " contour area " + str(cv2.contourArea(c)))
        if cv2.contourArea(c) < 1000:
            continue
        (x, y, w, h) = cv2.boundingRect(c)
        if w < h:
            # print("w: " + str(w) + " h:" + str(h))
            b_rects.append([x, y, w + x, h + y])
    
    max_box = findMaxHeight(b_rects)
    cv2.rectangle(frame, (max_box[0],max_box[1]), (max_box[2],max_box[3]), (0, 238, 255), 2)
    # print("run " + str(i) + " " + str(b_rects))
    cv2.imwrite("./results/frame_with_boxes_%d.jpg" % i, frame)
    
    cv2.drawContours(frame, b_contours, -1, (0,255,0), 3)
    cv2.imwrite("./results/frame_contours_maybe_%d.jpg" % i, frame)
    b_rects = []
    i = i +1