import cv2
import rospy
import time
import os
import cv_bridge
from sensor_msgs.msg import Image
import datetime

cam = cv2.VideoCapture(0)
bridge = cv_bridge.CvBridge()
cv2.namedWindow("test")
cv2.startWindowThread()

rospy.init_node('topic_publisher')
pub = rospy.Publisher('imgs', Image, queue_size=1)
rate = rospy.Rate(2)
img_counter = 0

save_dir = None

while not rospy.is_shutdown():
    ret, frame = cam.read()

    if (img_counter % 3) == 0:
        if not ret:
            break
        if save_dir is None:
            save_dir = "./pict_output_{}".format(datetime.datetime.now().strftime("%Y-%m-%d %H %M %S"))
            os.mkdir(save_dir)
        img_name = save_dir + "{}/opencv_frame_{}.jpg".format(save_dir, img_counter)
        cv2.imwrite(img_name, frame)
    msg = bridge.cv2_to_imgmsg(frame)
    pub.publish(msg)
    print("publishing {}".format(img_counter))


    img_counter += 1
    cv2.imshow("test", frame)
    cv2.waitKey(1)




    '''
    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
    '''
    # rate.sleep()

cam.release()

cv2.destroyAllWindows()
