#!/usr/bin/env python2

from StateMachine.sub import *
import math


#defines test state
class NoahTestSpin(Sub):

    #This gives the list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate'])

    
    def execute(self, userdata):
        self.init_state()

        #init_heading = gbl.init_heading

        gbl.state_heading = gbl.heading
        msg = self.init_joy_msg()

        magnitude = 1

        msg.axes[const.AXES['rotate']] = 0.3

        rospy.loginfo("just trying to spin")
        while(1):
            self.publish_joy(msg)

            angle = self.angle_diff(gbl.heading, gbl.state_heading)
            frontback = math.cos(anlge) * magnitude
            strafe = math.sin(angle) * magnitude
            msg.axes[const.AXES['frontback']] = frontback
            msg.axes[const.AXES['strafe']] = strafe

            rospy.sleep(const.SLEEP_TIME)
            
        print("wow")

        return 'through_gate'
