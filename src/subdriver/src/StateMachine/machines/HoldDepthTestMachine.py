#!/usr/bin/env python2

import rospy
import smach

from StateMachine.test_states.dive_test_state import *
from StateMachine.test_states.test_hold_depth import *
from StateMachine.taskless.dumb_start import *


def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['finished'])

    # Open the container
    with sm_AUV:

        rospy.loginfo("STATE MACHINE")
        smach.StateMachine.add('DUMB_START', Dumb_Start(), transitions={'setup_complete':'DIVE'})
        smach.StateMachine.add('DIVE', DiveDown(), transitions={'dove' : 'HOLD_DEPTH'})
        smach.StateMachine.add('HOLD_DEPTH', HoldDepth(), transitions={'dead' : 'finished'})

    # Execute SMACH plan
    outcome = sm_AUV.execute()

def main():
    createStateMachine()


if __name__ == '__main__':
    main()
