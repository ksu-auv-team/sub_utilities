#!/usr/bin/env python2

from StateMachine.sub import *
import math


#defines test state
class NoahTestSpin(Sub):

    #This gives the list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate'])

    def calc_frontback(self, angle, magnitude):
        frontback = 0.0
        frontback = math.cos(math.radians(angle)) * magnitude
        return frontback

    def calc_strafe(self, angle, magnitude):
        strafe = 0.0
        strafe = math.sin(math.radians(angle)) * magnitude
        return strafe
    
    def execute(self, userdata):
        self.init_state()

        initial_heading = gbl.init_heading

        gbl.state_heading = gbl.heading
        msg = self.init_joy_msg()

        #magnitude is the hypot
        magnitude = .07

        msg.axes[const.AXES['rotate']] = -0.2

        rospy.loginfo("just trying to spin")
        while(1):
            #Idk which heading to use RIP
            angle = self.angle_diff(gbl.init_heading, gbl.heading)
            if angle < 0:
                  angle += 360
            rospy.loginfo("Angle " + str(angle))
            frontback = self.calc_frontback(angle, magnitude)
            strafe = self.calc_strafe(angle, magnitude)
            rospy.loginfo("FB " + str(frontback))
            rospy.loginfo("strafe " + str(strafe))
            msg.axes[const.AXES['frontback']] = frontback
            msg.axes[const.AXES['strafe']] = strafe
            #rospy.loginfo(magnitude)
            self.publish_joy(msg)

            rospy.sleep(const.SLEEP_TIME)
            
    

        return 'through_gate'
