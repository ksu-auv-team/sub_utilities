#!/usr/bin/env python

from StateMachine.search.search_front import *

# define state search_front_dice
class Search_Front_Buoy(Search_Front):
	pass
	#def __init__(self):
	#	smach.State.__init__(self, outcomes = 'buoy_found', 'buoy_hit')

	def log(self):
		rospy.loginfo('Executing state SEARCH_FRONT_BUOY')
		
