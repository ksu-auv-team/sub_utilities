#!/usr/bin/env python2

from StateMachine.sub import *

#defines test state
class NoahTestSpin(Sub):

    #This gives the list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate'])

    
    def execute(self, userdata):
        self.init_state()

        gbl.state_heading = gbl.heading
        msg = self.init_joy_msg()

        msg.axes[const.AXES['rotate']] = -0.5

        rospy.loginfo("just trying to spin")
        while(1):
            self.publish_joy(msg)



        
        print("wow")

        return 'through_gate'
