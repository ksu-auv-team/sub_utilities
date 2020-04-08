#!/usr/bin/env python2

from StateMachine.search.search_front import *

# define state search_front_gate
class Search_Front_Gate(Search_Front):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_FRONT_GATE')