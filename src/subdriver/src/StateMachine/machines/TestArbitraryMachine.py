#!/usr/bin/env python2

import rospy
import smach

from StateMachine.taskless.dumb_start import *
from StateMachine.taskless.surface import *

def createStateMachine(arbitrary_state=None):
    rospy.init_node('AUV_StateMachine')
    
    if arbitrary_state is None:
        rospy.loginfo("Please provide the state you want to run.")
        return

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['finished_run'])
    
    # Open the container
    with sm_AUV:

        smach.StateMachine.add('DUMB_START', Dumb_Start(), transitions={'setup_complete':'ARBITRARY_STATE'})

        # Get every possible outcome in the arbitrary state and make it transition to SURFACE
        arbitrary_state_outcome_dict = {}
        for outcome in arbitrary_state.get_registered_outcomes():
            arbitrary_state_outcome_dict[outcome] = "SURFACE"

        smach.StateMachine.add('ARBITRARY_STATE', arbitrary_state, transitions=arbitrary_state_outcome_dict)

        smach.StateMachine.add('SURFACE', Surface(), transitions={'surfaced':'finished_run'})


    # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
