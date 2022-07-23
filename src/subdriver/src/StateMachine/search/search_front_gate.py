#!/usr/bin/env python2

from turtle import forward
from StateMachine.search.search_front import *

# define state search_front_gate
# We inherit from search_front state now so we gucci
class Search_Front_Gate(Search_Front):

	def log(self):
		rospy.loginfo('Executing state SEARCH_FRONT_GATE')