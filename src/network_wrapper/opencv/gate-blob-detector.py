#!/usr/bin/env python2

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
import time
import cv_bridge
import math
from submarine_msgs_srvs.msg import Detections
import argparse

def midpoint (p1, p2):
    return p1[0] + p2[0] * 0.5, p1[1] + p2[1] * 0.5

def distance (p1, p2):
    return math.sqrt(((p2[0] - p1[0]) ** 2) + ((p2[1] - p1[1]) ** 2))

def raw_img_callback(msg):
    image = bridge.imgmsg_to_cv2(msg)

    # print('image received', time.time())

    # BGR values
    orange_lower = np.array([0, 40, 80])
    orange_upper = np.array([67, 180, 255])

    black_lower = np.array([0, 0, 0])
    black_upper = np.array([50, 40, 40])


    #get posts
    orange_mask = cv2.inRange(image, orange_lower, orange_upper)

    orange_mask = cv2.erode(orange_mask, None, iterations=2)
    orange_mask = cv2.dilate(orange_mask, None, iterations=2)

    #get crossbar
    black_mask = cv2.inRange(image, black_lower, black_upper)
    black_mask = cv2.erode(black_mask, None, iterations=2)
    black_mask = cv2.dilate(black_mask, None, iterations=2)

    # get contours
    o_img, o_contours, o_hierarchy = cv2.findContours(orange_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    b_img, b_contours, b_hierarchy = cv2.findContours(black_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    # check which contours we want and convert to rectangles
    b_rects = []
    o_rects = []
    for c in o_contours:
        if cv2.contourArea(c) < 3000:
            continue


        #top left point, plus width and height
        (x, y, w, h) = cv2.boundingRect(c)

        #double-check rectangle proportions to make sure there's nothing screwy going on
        if not h > w:
            o_rects.append([x, y, w + x, h + y])
            cv2.rectangle(image, (x,y), (x+w,y+h), (0, 128, 255), 2)

    for c in b_contours:
        if cv2.contourArea(c) < 3000:
            continue

        (x, y, w, h) = cv2.boundingRect(c)
        
        #double-check rectangle proportions to make sure there's nothing screwy going on
        # if not w < 5 * h:
        b_rects.append([x, y, w + h, h + y])
        cv2.rectangle(image, (x,y), (x+w,y+h), (0, 255, 0), 2)
        
    #could check something about relative position of black and orange boxes here - may do that later
    #i.e. make sure the posts are actually below the ends of the crossbar


    print(o_rects)
    print(b_rects)
    #combine two lists into one
    rects = o_rects
    rects.extend(b_rects)

    img_h, img_w, img_chan = image.shape

    #convert to neural network coordinate system 
    #top left of img is origin, bottom right is (1, 1)
    for r in rects:
        r[0] = r[0] / img_w
        r[1] = r[1] / img_h
        r[2] = r[2] / img_w
        r[3] = r[3] / img_h
        #do this in a second once tyler pushes code


    #publish image & detection
    img_msg = bridge.cv2_to_imgmsg(image)
    img_pub.publish(img_msg)

    #combine bounding boxes into larger bounding box
    if rects:
        x_min = min([box[0] for box in rects])
        y_min = min([box[1] for box in rects])
        x_max = max([box[2] for box in rects])
        y_max = max([box[3] for box in rects])
        full_box = [x_min, y_min, x_max, y_max]

        detections_msg = Detections()
        detections_msg.scores = [1.0]
        detections_msg.boxes = [full_box]
        detections_msg.classes = [args.class_num]
        detections_msg.detected = [1]
        det_pub.publish(detections_msg)

    if args.show or args.debug:
        cv2.imshow('gate_blob_detector', image)
        cv2.waitKey(1)

    print('image processed', time.time())

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script gets a bounding box from an orange thing. Intended to be used for testing.")
    parser.add_argument('-c', '--camera', default='0', type=int, help='int indicating to get images from front camera (0) or bottom camera (1). Default is 0')
    parser.add_argument('-n', '--class_num', default="8", type=int, help="index of class that will be returned (default is 8, which is the sgate)")
    parser.add_argument('-b', '--bottom', action='store_true', help='send images on bottom camera topic instead of front camera topic')
    parser.add_argument('-s', '--show', action='store_true', help='display images locally with opencv')
    parser.add_argument('-d', '--debug', action='store_true', help='print more and display all bounding boxes locally')

    args = parser.parse_args()

    if args.bottom:
        img_pub = rospy.Publisher('bottom_network_imgs', Image, queue_size=1)
        det_pub = rospy.Publisher('bottom_network_output', Detections, queue_size=1)
    else:
        img_pub = rospy.Publisher('front_network_imgs', Image, queue_size=1)
        det_pub = rospy.Publisher('front_network_output', Detections, queue_size=1)

    bridge = cv_bridge.CvBridge()

    if args.camera == 1: 
        rospy.Subscriber('bottom_raw_imgs', Image, raw_img_callback, queue_size=1)
    else:
        rospy.Subscriber('front_raw_imgs', Image, raw_img_callback, queue_size=1)

    print('started')

    rospy.init_node('test_detection_node', anonymous=True)
    rospy.spin()