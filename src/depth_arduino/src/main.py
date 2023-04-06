#!/usr/bin/env python2
import rospy
import depth

def startup():
    pub = rospy.Publisher('depth', String, queue_size=10)#Replace String and maybe change quesize
    rospy.init_node("depth_arduino")
    rate = rospy.Rate(0.0166667) #60 seconds, maybe change later
    rospy.loginfo("node depth_arduino has been started.")
    
    while not rospy.is_shutdown():
        data = depth
        depth = data.get_depth()
        temp = data.get_temp()
        pub.publish(depth, temp)
        rate.sleep()


if __name__ == '__main__':
    try:
        startup()
    except rospy.ROSInterruptException:
        pass