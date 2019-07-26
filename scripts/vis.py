import cv2
import rospy
from sensor_msgs.msg import Image
import time 
import cv_bridge
import argparse

bridge = cv_bridge.CvBridge()
def front_raw_img_callback(msg):
    img = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('front_raw_img', img)
    cv2.waitKey(1)

def front_network_img_callback(msg):
    img = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('front_network_img', img)
    cv2.waitKey(1)

def bottom_raw_img_callback(msg):
    img = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('bottom_raw_img', img)
    cv2.waitKey(1)

def bottom_network_img_callback(msg):
    img = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('bottom_network_img', img)
    cv2.waitKey(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script pulls images over ros topic and displays them to the local computer")
    parser.add_argument('-n', '--network', action='store_true', help='Will listen to the network_imgs topic and display images')
    parser.add_argument('-r', '--raw', action='store_true', help='Will listen to the raw_imgs topic and display images')
    args = parser.parse_args()

    if(args.network):
        rospy.Subscriber('front_network_imgs', Image, front_network_img_callback, queue_size=1)
        rospy.Subscriber('bottom_network_imgs', Image, bottom_network_img_callback, queue_size=1)

    if(args.raw):
        rospy.Subscriber('front_raw_imgs', Image, front_raw_img_callback, queue_size=1)
        rospy.Subscriber('bottom_raw_imgs', Image, bottom_raw_img_callback, queue_size=1)
    
    if(not args.raw and not args.network):
        print("Please specify -r for raw images, -n for network images, or both to display both")
        exit()

    rospy.init_node('vis_node', anonymous=True)
    rospy.spin()

cv2.destroyAllWindows()