#!/usr/bin/env python

import rospy
import smach

#import literally all the states we will ever have:
from StateMachine.interact.interact_gate import *
from StateMachine.interact.interact_pole import *

from StateMachine.search.search_front import *
from StateMachine.search.search_left import *
from StateMachine.search.search_right import *
from StateMachine.search.search_recenter import *

from StateMachine.search.search_front_gate import *
from StateMachine.search.search_left_gate import *
from StateMachine.search.search_right_gate import *
from StateMachine.search.search_recenter_gate import *

from StateMachine.search.search_front_pole import *
from StateMachine.search.search_left_pole import *
from StateMachine.search.search_right_pole import *
from StateMachine.search.search_recenter_pole import *

from StateMachine.track.track_gate import *
from StateMachine.track.track_pole import *

from StateMachine.taskless.start import *
from StateMachine.taskless.surface import *

def createStateMachine():
    rospy.init_node('AUV_StateMachine')

    # Create the top level SMACH state machine
    sm_AUV = smach.StateMachine(outcomes=['Finished_Run'])
    
    # Open the container
    with sm_AUV:

        smach.StateMachine.add('START', Start(), transitions={'Not_Found_Gate':'SEARCH_FRONT_GATE', 'Found_Gate':'TRACK_GATE'})

        smach.StateMachine.add('SEARCH_FRONT_GATE', Search_Front_Gate(), transitions={'object_found':'TRACK_GATE', 'object_not_found':'SEARCH_LEFT_GATE'})
        smach.StateMachine.add('SEARCH_LEFT_GATE', Search_Left_Gate(), transitions={'object_found':'TRACK_GATE', 'object_not_found':'SEARCH_RIGHT_GATE'})
        smach.StateMachine.add('SEARCH_RIGHT_GATE', Search_Right_Gate(), transitions={'object_found':'TRACK_GATE', 'object_not_found':'SEARCH_RECENTER_GATE'})
        smach.StateMachine.add('SEARCH_RECENTER_GATE', Search_Recenter_Gate(), transitions={'object_found':'TRACK_GATE', 'object_not_found':'SEARCH_FRONT_GATE'})

        smach.StateMachine.add('TRACK_GATE', Track_Gate(), transitions={'Lost_Gate':'SEARCH_FRONT_GATE', 'Approached_Gate':'INTERACT_GATE'})

        smach.StateMachine.add('INTERACT_GATE', Interact_Gate(), transitions={'Through_Gate':'SEARCH_FRONT_POLE'})
        
        smach.StateMachine.add('SEARCH_FRONT_POLE', Search_Front_Pole(), transitions={'object_found':'TRACK_POLE', 'object_not_found':'SEARCH_LEFT_POLE'})
        smach.StateMachine.add('SEARCH_LEFT_POLE', Search_Left_Pole(), transitions={'object_found':'TRACK_POLE', 'object_not_found':'SEARCH_RIGHT_POLE'})
        smach.StateMachine.add('SEARCH_RIGHT_POLE', Search_Right_Pole(), transitions={'object_found':'TRACK_POLE', 'object_not_found':'SEARCH_RECENTER_POLE'})
        smach.StateMachine.add('SEARCH_RECENTER_POLE', Search_Recenter_Pole(), transitions={'object_found':'TRACK_POLE', 'object_not_found':'SEARCH_FRONT_POLE'})        

        smach.StateMachine.add('TRACK_POLE', Track_Pole(), transitions={'Lost_Pole':'SEARCH_FRONT_POLE', 'Approached_Pole':'INTERACT_POLE'})

        smach.StateMachine.add('INTERACT_POLE', Interact_Pole(), transitions={'Around_Pole':'SEARCH_FRONT_GATE', 'Lost_Pole':'SEARCH_FRONT_POLE'})

        smach.StateMachine.add('SURFACE', Surface(), transitions={'Surfaced':'Finished_Run'})

        # Execute SMACH plan
    outcome = sm_AUV.execute()



def main():
    createStateMachine()


if __name__ == '__main__':
    main()
