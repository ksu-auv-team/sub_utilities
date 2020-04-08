#!/usr/bin/env python

from  StateMachine.search.search_right import *

# define state search_right_pole
class Search_Right_Pole(Search_Right):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_RIGHT_POLE')