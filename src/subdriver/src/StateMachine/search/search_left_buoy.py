#!/usr/bin/env python

from StateMachine.search.search_left import *

# define state search_left_dice
class Search_Left_Buoy(Search_Left):
	pass

	def log(self):
		rospy.loginfo('Executing state SEARCH_LEFT_BUOY')