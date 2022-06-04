#!/usr/bin/env python

import rospy
import smach

from StateMachine.test_states.dive_test_state import *
from StateMachine.test_states.surface_test_state import *
from StateMachine.test_states.forward_test_state import *
from StateMachine.test_states.backwards_test_state import *
from StateMachine.test_states.rotate_left_test_state import *
from StateMachine.test_states.rotate_right_test_state import *
from StateMachine.test_states.strafe_left_test_state import *
from StateMachine.test_states.strafe_right_test_state import *

def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['surfaced'])
    
    # Open the container
    with sm_AUV:

        smach.StateMachine.add('DIVE', DiveDown(), transitions={'dove' : 'FORWARD'})

        smach.StateMachine.add('FORWARD', Forward(), transitions={'went_forward' : 'BACKWARDS'})
        smach.StateMachine.add('BACKWARDS', Backwards(), transitions={'went_backwards':'STRAFE_LEFT'})

        smach.StateMachine.add('STRAFE_LEFT', StrafeLeft(), transitions={'strafed_left':'STRAFE_RIGHT'})
        smach.StateMachine.add('STRAFE_RIGHT', StrafeRight(), transitions={'strafed_right':'ROTATE_LEFT'})

        smach.StateMachine.add('ROTATE_LEFT', RotateLeft(), transitions={'rotated_left' : 'ROTATE_RIGHT'})
        smach.StateMachine.add('ROTATE_RIGHT', RotateLeft(), transitions={'rotated_right' : 'SURFACE'})

        smach.StateMachine.add('SURFACE', SurfaceSub(), transitions={'gone_up' : 'surfaced'})

    # Execute SMACH plan
    outcome = sm_AUV.execute()

def main():
    createStateMachine()


if __name__ == '__main__':
    main()
