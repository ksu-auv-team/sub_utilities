#!/usr/bin/env python
import rospy
from submarine_msgs_srvs.msg import Detections

def callback(data):
    rospy.loginfo("scores: ")
    rospy.loginfo(data.scores)

    rospy.loginfo("boxes:")
    rospy.loginfo(data.boxes)

    rospy.loginfo("classes:")
    rospy.loginfo(data.classes)


    rospy.loginfo("detected:")
    rospy.loginfo(data.detected)


def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("custom_chatter", Detections, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  
if __name__ == '__main__':
    listener()