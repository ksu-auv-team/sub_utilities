#!/usr/bin/env python

from StateMachine.search.search_recenter import *

# define state search_recenter_gate
class Search_Recenter_Gate(Search_Recenter):
    pass


    def log(self):
        rospy.loginfo('Executing state SEARCH_RECENTER_GATE')