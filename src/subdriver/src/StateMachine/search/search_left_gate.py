#!/usr/bin/env python2

from StateMachine.search.search_left import *

# define state Search_Left_gate
class Search_Left_Gate(Search_Left):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_LEFT_GATE')