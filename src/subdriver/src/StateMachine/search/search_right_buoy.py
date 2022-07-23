#!/usr/bin/env python

from StateMachine.search.search_right import *

# define states
class Search_Right_Buoy(Search_Right):
    pass

    def log(self):
        rospy.loginfo('Executing state SEARCH_RIGHT_BUOY')