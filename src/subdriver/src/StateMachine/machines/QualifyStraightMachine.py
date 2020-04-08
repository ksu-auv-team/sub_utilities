#!/usr/bin/env python

import rospy
import smach

#import literally all the states we will ever have:
from StateMachine.search.search_front import *
from StateMachine.search.search_left import *
from StateMachine.search.search_right import *
from StateMachine.search.search_recenter import *

from StateMachine.search.search_front_gate import *
from StateMachine.search.search_left_gate import *
from StateMachine.search.search_right_gate import *
from StateMachine.search.search_recenter_gate import *

from StateMachine.taskless.dumb_start import *
from StateMachine.taskless.surface import *
from StateMachine.taskless.straight_ahead import *

def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['Finished_Run'])
    sm_gate_search = smach.StateMachine(outcomes=['search_found'])

    # Open the container
    with sm_AUV:

        smach.StateMachine.add('DUMB_START', Dumb_Start(), transitions={'setup_complete':'STRAIGHT_AHEAD'})

        with sm_gate_search:
            smach.StateMachine.add('SEARCH_FRONT_GATE', Search_Front_Gate(), transitions={'object_found':'SEARCH_LEFT_GATE', 'object_not_found':'SEARCH_LEFT_GATE'})
            smach.StateMachine.add('SEARCH_LEFT_GATE', Search_Left_Gate(), transitions={'object_found':'SEARCH_RIGHT_GATE', 'object_not_found':'SEARCH_RIGHT_GATE'})
            smach.StateMachine.add('SEARCH_RIGHT_GATE', Search_Right_Gate(), transitions={'object_found':'SEARCH_RECENTER_GATE', 'object_not_found':'SEARCH_RECENTER_GATE'})
            smach.StateMachine.add('SEARCH_RECENTER_GATE', Search_Recenter_Gate(), transitions={'object_found':'search_found', 'object_not_found':'search_found'})

        smach.StateMachine.add('SEARCH_GATE', sm_gate_search, transitions={'search_found':'STRAIGHT_AHEAD'})
        smach.StateMachine.add('STRAIGHT_AHEAD', Straight_Ahead(), transitions={'through_gate':'SURFACE'})
        smach.StateMachine.add('SURFACE', Surface(), transitions={'surfaced':'Finished_Run'})

        # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
