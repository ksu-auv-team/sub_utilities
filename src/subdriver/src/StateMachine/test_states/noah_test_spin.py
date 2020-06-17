#!/usr/bin/env python2

from StateMachine.sub import *

#defines test state
class NoahTestSpin(Sub):

    #This gives the list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate'])

    
    def execute(self, userdata):
        self.init_state()

        rospy.loginfo("wow")
        print("wow")

        return 'through_gate'
