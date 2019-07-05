 #!/usr/bin/env python
import rospy
from submarine_msgs_srvs.msg import Detections

def talker():
    pub = rospy.Publisher('custom_chatter', Detections)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = Detections()
    msg.scores = [0, 1, 2, 3]
    msg.boxes = [4, 5, 6, 7]
    msg.classes = [8, 9, 10, 11]
    msg.detected = [12]

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()
  
if __name__ == '__main__':
    try:
            talker()
    except rospy.ROSInterruptException: pass