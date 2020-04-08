#!/usr/bin/env python

from StateMachine.search.search_left import *

# define state Search_Left_pole
class Search_Left_Pole(Search_Left):
    pass

    def log(self):
        rospy.loginfo('Executing state SEARCH_LEFT_POLE')