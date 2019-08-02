#!/usr/bin/env python2

import cv2
import rospy
import os
import cv_bridge
from sensor_msgs.msg import Image
import argparse
from random import shuffle

def main():
    rospy.init_node('Sub_Video_Publish')
    bridge = cv_bridge.CvBridge()

    img_counter = 0

    if args.topic == 0:
        pub = rospy.Publisher('front_raw_imgs', Image)
    else:
        pub = rospy.Publisher('bottom_raw_imgs', Image)

    #get files to load
    img_dir = args.img_path

    filenames = os.listdir(img_dir)

    if args.shuffle:
        shuffle(filenames)

    for filename in os.listdir(img_dir):
        image = cv2.imread(os.path.join(img_dir, filename))
        print(filename)

        msg = bridge.cv2_to_imgmsg(image)
        pub.publish(msg)

        if (args.show_video):
            cv2.imshow('img_dir_pub', image)
            if args.fps == 0:
                cv2.waitKey(0)
            else:
                cv2.waitKey(1000/args.fps)
        else:
            rospy.sleep(1.0/args.fps)



    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Parse command line arguments:
    parser = argparse.ArgumentParser(description="Get video from the sub over a ros topic")
    parser.add_argument('-p', '--img_path', help='Path to the folder of images to read. Required.')
    parser.add_argument('-v', '--show_video', action='store_true', help='Will show the video onboard with opencv')
    parser.add_argument('-t', '--topic', type=int, default='0', help='int representing topic to publish images to. 0 is front cam, 1 is bottom cam.s')
    parser.add_argument('-f', '--fps', type=int, default='10', help='number of frames per second to publish. Entering 0 with -v will set opencv to continue/publish only on a keypress. Default is 10.')
    parser.add_argument('-s', '--shuffle', action='store_true', help='shuffle images')
    #disabled for now - may reimplement these later to resize images
    # parser.add_argument('--front-height', default=420, type=int, help='Set the front video capture height for your camera in pixels')
    # parser.add_argument('--bottom-height', default=420, type=int, help='Set the bottom video capture height for your camera in pixels')
    # parser.add_argument('--front-width', default=860, type=int, help='Set the front video capture width for your camera in pixels')
    # parser.add_argument('--bottom-width', default=860, type=int, help='Set the bottom video capture width for your camera in pixels')
    args = parser.parse_args()

    if not args.img_path:
        print("Must provide directory to load images from with -p")
        exit()

    if args.fps == 0 and not args.show_video:
        print("Cannot use 0 FPS without -v")
        exit()

    main()
