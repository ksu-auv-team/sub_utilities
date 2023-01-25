#!/usr/bin/env python3

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
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg)

    # rospy.loginfo('image received', time.time())

    # BGR values
    lower_marker = np.array([0, 40, 120])
    upper_marker = np.array([140, 180, 255])

    mask = cv2.inRange(image, lower_marker, upper_marker)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Finds contours
    im2, contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    x = None
    y = None
    w = None
    h = None

    # Draws contours
    for c in contours:
        if cv2.contourArea(c) < 3000:
            continue

        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(image, (x,y), (x+w,y+h), (0, 255, 0), 2)    

    if x is not None:
        #publish image & detection
        img_msg = bridge.cv2_to_imgmsg(image)
        img_pub.publish(img_msg)

        img_h, img_w, img_chan = image.shape

        detections_msg = Detections()
        detections_msg.scores = [1]
        detections_msg.boxes = [x/img_w, y/img_h, (x + w) / img_w, (y + h) / img_h]
        detections_msg.classes = [args.class_num]
        detections_msg.detected = [1]
        det_pub.publish(detections_msg)

    # cv2.imshow('arms', image)
    # cv2.waitKey(1)
    if args.show:
        cv2.imshow('marker-detection', image)
        cv2.waitKey(1)

    rospy.loginfo('image processed', time.time())

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script gets a bounding box from an orange thing. Intended to be used for testing.")
    parser.add_argument('-c', '--camera', default='0', type=int, help='int indicating to use front camera (0) or bottom camera (1). Default is 0')
    parser.add_argument('-n', '--class_num', default="1", type=int, help="index of class that will be returned (default is 1)")
    parser.add_argument('-b', '--bottom', action='store_true', help='send images on bottom camera topic instead of front camera topic')
    parser.add_argument('-s', '--show', action='store_true', help='display images locally with opencv')

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

    rospy.init_node('test_detection_node', anonymous=True)
    rospy.loginfo('started')

    rospy.spin()
