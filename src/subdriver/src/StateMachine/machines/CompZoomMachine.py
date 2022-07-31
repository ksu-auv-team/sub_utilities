#!/usr/bin/env python2

import rospy
import smach

from StateMachine.test_states.forward_zoom import *
from StateMachine.test_states.spin_zoom import *
from StateMachine.test_states.surface_test_state import *
from StateMachine.test_states.temp_stop_state import *
from StateMachine.taskless.dumb_start import *


def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['finished_functions'])

    # Open the container
    with sm_AUV:

        rospy.loginfo("STATE MACHINE")
        smach.StateMachine.add('DUMB_START', Dumb_Start(), transitions={'setup_complete':'FORWARD'})
        smach.StateMachine.add('FORWARD', Forward(), transitions={'went_forward_zoom': 'SPIN'})
        smach.StateMachine.add('SPIN', Spin(), transitions={'spun_around': 'SURFACE'})
        smach.StateMachine.add('SURFACE', SurfaceSub(), transitions={'gone_up' : 'STOPPED'})
        smach.StateMachine.add('STOPPED', StopState(), transitions={'stopped':'finished_functions'})

    # Execute SMACH plan
    outcome = sm_AUV.execute()

def main():
    createStateMachine()


if __name__ == '__main__':
    main()
