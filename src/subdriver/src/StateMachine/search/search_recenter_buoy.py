#!/usr/bin/env python

from StateMachine.search.search_recenter import *

# define state search_recenter_dice
class Search_Recenter_Buoy(Search_Recenter):
    pass
    

    def log(self):
        rospy.loginfo('Executing state SEARCH_RECENTER_BUOY')