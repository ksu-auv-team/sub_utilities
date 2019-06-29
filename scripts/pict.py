import cv2
import rospy
import time
import os
import cv_bridge
from sensor_msgs.msg import Image
import datetime
import argparse

def main():
    cam = cv2.VideoCapture(0)
    bridge = cv_bridge.CvBridge()
    cv2.namedWindow("Sub_Video")
    cv2.startWindowThread()

    rospy.init_node('Sub_Video_Publish')
    pub = rospy.Publisher('imgs', Image, queue_size=1)
    rate = rospy.Rate(2)
    img_counter = 0

    # Get current script directory
    script_directory = os.path.dirname(os.path.realpath(__file__)) + '/'
    save_dir = None

    if (not args.no_save_images):
        save_dir = script_directory + '../saved_video/{}/'.format(datetime.datetime.now())
        os.mkdir(save_dir)

    while not rospy.is_shutdown():
        ret, frame = cam.read()

        if (img_counter % 3) == 0 and args.no_save_images:
            if not ret:
                break
            #if save_dir is None:
            #    save_dir = "./pict_output_{}".format(datetime.datetime.now().strftime("%Y-%m-%d %H %M %S"))
            #    os.mkdir(save_dir)
            img_name = save_dir + "{}/opencv_frame_{}.jpg".format(save_dir, img_counter)
            cv2.imwrite(img_name, frame)
        msg = bridge.cv2_to_imgmsg(frame)
        pub.publish(msg)
        print("publishing {}".format(img_counter))


        img_counter += 1
        cv2.imshow("Sub_Video", frame)
        cv2.waitKey(1)

    cam.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Parse command line arguments:
    parser = argparse.ArgumentParser(description="Get video from the sub over a ros topic")
    parser.add_argument('-s', '--no-save-images', action='store_true', help="Boolean flag to not save images to 'saved_video'")
    args = parser.parse_args()

    main()
