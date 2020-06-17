#!/usr/bin/env python2

import rospy
import smach

from StateMachine.test_states.noah_test_spin import *
from StateMachine.taskless.surface import *

def creatStateMachine():

    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['finished_run'])

    # Open the container
    with sm_AUV:
        smach.StateMachine.add("WOW", NoahTestSpin(), transitions={'through_gate': 'SURFACE'})
        smach.StateMachne.add("SURFACE", Surface(), transitions={'surfaced' : 'finished_run'})

    # Execute SMACH plan
    outcome = sm_AUV.execute()

def main():
    createStateMachine()


if __name__ == '__main__':
    main()