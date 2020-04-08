#!/usr/bin/env python2

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
import time
import cv_bridge
import math

def midpoint (p1, p2):
    return p1[0] + p2[0] * 0.5, p1[1] + p2[1] * 0.5

def distance (p1, p2):
    return math.sqrt(((p2[0] - p1[0]) ** 2) + ((p2[1] - p1[1]) ** 2))

def order_points(pts):
    #copied from the imutil library because I didn't want to install a library for a single function
    # sort the points based on their x-coordinates
    xSorted = pts[np.argsort(pts[:, 0]), :]

    # grab the left-most and right-most points from the sorted
    # x-roodinate points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]

    # now, sort the left-most coordinates according to their
    # y-coordinates so we can grab the top-left and bottom-left
    # points, respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost

    # now that we have the top-left coordinate, use it as an
    # anchor to calculate the Euclidean distance between the
    # top-left and right-most points; by the Pythagorean
    # theorem, the point with the largest distance will be
    # our bottom-right point
    dist0 = distance(tl, rightMost[0])
    dist1 = distance(tl, rightMost[1])

    if dist0 > dist1:
        br = rightMost[0]
        tr = rightMost[1]

    else:
        br = rightMost[1]
        tr = rightMost[0]

    # D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    # (br, tr) = rightMost[np.argsort(D)[::-1], :]

    # return the coordinates in top-left, top-right,
    # bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype="float32")
    

def bottom_raw_img_callback(msg):
    image = bridge.imgmsg_to_cv2(msg)

    # print('image received', time.time())

    # BGR values
    lower_marker = np.array([0, 40, 80])
    upper_marker = np.array([67, 180, 255])

    mask = cv2.inRange(image, lower_marker, upper_marker)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Finds contours
    im2, contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draws contours
    for c in contours:
        if cv2.contourArea(c) < 3000:
            continue

        # get rotated rectangle for the whole marker
        rect = cv2.minAreaRect(c)
        rect_box = cv2.boxPoints(rect) #order is bottom left, top left, top right, bottom right
        rect_box = np.int0(rect_box)

        #   Remove it from the image, crop, and rotate it


        #   Keep track of the angle for calculations later
        #   Split the rotated rectangle in half
        #   Run contour detection again on each half (or some code that will get the angle of each side)
        #   Use the angles of each side and the angle we rotated the image to calculate the sub's angle relative to the marker

        # get width and height of the detected rectangle
        width = int(rect[1][0])
        height = int(rect[1][1])
        src_pts = rect_box.astype("float32")
        # coordinate of the points in box points after the rectangle has been
        # straightened
        dst_pts = np.array([[0, height-1],
                            [0, 0],
                            [width-1, 0],
                            [width-1, height-1]], dtype="float32")

        # the perspective transformation matrix
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)

        # directly warp the rotated rectangle to get the straightened rectangle
        warped = cv2.warpPerspective(image, M, (width, height))

        if len(warped) > len(warped[0]): #if height greater than width
            #split horizontally
            img1 = warped[0:len(warped)/2, len(warped[0])]
            img2 = warped[len(warped)/2:len(warped), len(warped[0])]
        else:
            #split vertically
            img1 = warped[len(warped), 0:len(warped[0])/2]
            img2 = warped[len(warped), len(warped[0])/2:len(warped[0])]
            #if it's a square we're screwed for that frame

            #once we've split the image, get the angle of the arm in each side
            #do it by getting the minarearect and finding the angle from that
            c_img1, contours1, hierarchy1 = cv2.findContours(img1.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours1:
                if cv2.contourArea(c) < 3000:
                    continue

                 # get rotated rectangle for the whole marker
                rect = cv2.minAreaRect(c)
                rect_box = cv2.boxPoints(rect) #order is bottom left, top left, top right, bottom right
                rect_box = np.int0(rect_box)
                
                

        # get rotated rectangle for the whole marker
        rect = cv2.minAreaRect(c)
        rect_box = cv2.boxPoints(rect) #order is bottom left, top left, top right, bottom right
        rect_box = np.int0(rect_box)

        #find the difference between the angles
        if cv2.contourArea(c) < 3000:
            continue

        # get rotated rectangle for the whole marker
        rect = cv2.minAreaRect(c)
        rect_box = cv2.boxPoints(rect) #order is bottom left, top left, top right, bottom right
        rect_box = np.int0(rect_box)

        #determine angle
        if rect[1][0] > rect[1][1]:
            ang = (rect[2] + 90)* np.pi / 180
        else:
            ang = rect[2]* np.pi / 180
        rot = np.matrix([[np.cos(ang), -np.sin(ang)],[np.sin(ang), np.cos(ang)]])
        rv = rot*v

        print(ang)
        print (rot)
        print(rv)


        #use the angles, the difference, and the number of degrees rotated to get the angles of the arms

    
    # cv2.imshow('arms', image)
    # cv2.waitKey(1)
    cv2.imshow('marker-detection', warped)
    cv2.waitKey(1)
    print('showed')

    # print('image processed', time.time())



bridge = cv_bridge.CvBridge()
rospy.Subscriber('bottom_raw_imgs', Image, bottom_raw_img_callback, queue_size=1)
print('started')

rospy.init_node('arms_node', anonymous=True)
rospy.spin()