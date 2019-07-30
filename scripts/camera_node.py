#!/usr/bin/env python2

import cv2
import rospy
import time
import os
import cv_bridge
from sensor_msgs.msg import Image
import datetime
import argparse

def main():
    if (not args.debug):
        rospy.init_node('Sub_Video_Publish')
        bridge = cv_bridge.CvBridge()

    if not args.no_front:
        front_cam = cv2.VideoCapture(args.front_camera)
        front_cam.set(3, args.front_width)
        front_cam.set(4, args.front_height)
        if (not args.debug):
            front_pub = rospy.Publisher('front_raw_imgs', Image, queue_size=1)

    if not args.no_bottom:
        bottom_cam = cv2.VideoCapture(args.bottom_camera)
        bottom_cam.set(3, args.bottom_width)
        bottom_cam.set(4, args.bottom_height)
        if (not args.debug):
            bottom_pub = rospy.Publisher('bottom_raw_imgs', Image, queue_size=1)
    
    img_counter = 0

    # Get current script directory
    script_directory = os.path.dirname(os.path.realpath(__file__)) + '/'
    save_dir = None

    if (not args.no_save_images):
        save_dir = script_directory + '../saved_video/{}/'.format(datetime.datetime.now())
        print(save_dir)
        os.mkdir(save_dir)

    while not rospy.is_shutdown():
        front_ret = False
        bottom_ret = False

        if not args.no_front:
            front_ret, front_frame = front_cam.read()

        if not args.no_bottom:
            bottom_ret, bottom_frame = bottom_cam.read()

        if (img_counter % 3) == 0 and not args.no_save_images:
            if not front_ret and not bottom_ret:
                break
            front_img_name = "{}/front_opencv_frame_{}.jpg".format(save_dir, img_counter)
            bottom_img_name = "{}/bottom_opencv_frame_{}.jpg".format(save_dir, img_counter)
            if not args.no_front:
                cv2.imwrite(front_img_name, front_frame)
            if not args.no_bottom:
                cv2.imwrite(bottom_img_name, bottom_frame)

        if(not args.debug):
            if not args.no_front and front_ret:
                front_msg = bridge.cv2_to_imgmsg(front_frame)
                front_pub.publish(front_msg)
                print("publishing front {}".format(img_counter))
            if not args.no_bottom and bottom_ret:
                bottom_msg = bridge.cv2_to_imgmsg(bottom_frame)
                bottom_pub.publish(bottom_msg)
                print("publishing bottom {}".format(img_counter))

        img_counter += 1

        if(args.show_video and (front_ret or bottom_ret)):
            if not args.no_front:
                cv2.imshow("Sub_Front_Video", front_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            if not args.no_bottom:
                cv2.imshow("Sub_Bottom_Video", bottom_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    if not args.no_front:
        front_cam.release()
    if not args.no_bottom:
        bottom_cam.release()


    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Parse command line arguments:
    parser = argparse.ArgumentParser(description="Get video from the sub over a ros topic")
    parser.add_argument('-s', '--no-save-images', action='store_true', help="Boolean flag to not save images to 'saved_video'")
    parser.add_argument('-d', '--debug', action='store_true', help='Wll not run the ros stuff, allows for just running the video')
    parser.add_argument('-v', '--show-video', action='store_true', help='Will show the video onboard with opencv')
    parser.add_argument('-f', '--front-camera', default='/dev/video0', help='/path/to/video, defaults to /dev/video0')
    parser.add_argument('-b', '--bottom-camera', default='/dev/video2', help='/path/to/video, defaults to /dev/video2')
    parser.add_argument('--no-front', action='store_true', help='Will not open the front camera')
    parser.add_argument('--no-bottom', action='store_true', help='Will not open the bottom camera')
    parser.add_argument('--front-height', default=420, type=int, help='Set the front video capture height for your camera in pixels')
    parser.add_argument('--bottom-height', default=420, type=int, help='Set the bottom video capture height for your camera in pixels')
    parser.add_argument('--front-width', default=860, type=int, help='Set the front video capture width for your camera in pixels')
    parser.add_argument('--bottom-width', default=860, type=int, help='Set the bottom video capture width for your camera in pixels')
    args = parser.parse_args()

    if args.no_front and args.no_bottom:
        print("You must use at least one camera!")
        exit()

    main()
