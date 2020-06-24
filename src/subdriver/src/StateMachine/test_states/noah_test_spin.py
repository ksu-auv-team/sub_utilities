#!/usr/bin/env python2

from StateMachine.sub import *
import math


#defines test state
class NoahTestSpin(Sub):

    #This gives the list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate'])

    def calc_frontback(self, angle):
        frontback = 0.0
        frontback = math.sin(angle) * 0.3
        return frontback

    def calc_strafe(self, angle):
        strafe = 0.0
        strafe = math.cos(angle) * 0.3
        return strafe
    
    def execute(self, userdata):
        self.init_state()

        initial_heading = gbl.init_heading

        gbl.state_heading = gbl.heading
        msg = self.init_joy_msg()

        magnitude = 0.3
        frontback = 0.0
        strafe = 0.0
        msg.axes[const.AXES['rotate']] = 0.3

        rospy.loginfo("just trying to spin")
        while(1):
            angle = self.angle_diff(gbl.init_heading, gbl.heading)
            frontback = self.calc_frontback(angle)
            strafe = self.calc_strafe(angle)
            msg.axes[const.AXES['frontback']] = frontback
            msg.axes[const.AXES['strafe']] = strafe
            self.publish_joy(msg)

            rospy.sleep(const.SLEEP_TIME)
            
        print("wow")

        return 'through_gate'
