#!/usr/bin/env python

from  StateMachine.search.search_right import *

# define state search_right_gate
class Search_Right_Gate(Search_Right):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_RIGHT_GATE')