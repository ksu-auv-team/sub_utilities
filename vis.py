import cv2
import rospy
from sensor_msgs.msg import Image
import time 
import cv_bridge

bridge = cv_bridge.CvBridge()
def callback(msg):
    img = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('vis', img)
    cv2.waitKey(1)
    print('image recieved', time.time())

if __name__ == "__main__":
    rospy.Subscriber('imgs', Image, callback)
    rospy.init_node('vis_node', anonymous=True)
    rospy.spin()

cv2.destroyAllWindows()