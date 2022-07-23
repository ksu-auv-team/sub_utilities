#!/usr/bin/env python

from StateMachine.search.search_front import *

# define state search_front_buoy
# We inherit from search_front state now so we gucci
class Search_Front_Buoy(Search_Front):
	#def __init__(self):
	#	smach.State.__init__(self, outcomes = 'buoy_found', 'buoy_hit')
	# Gives a list of outcomes from this state
	# def __init__(self):
	# 	smach.State.__init__(self,outcomes=['object_found', 'object_not_found'])

	# def execute(self, userdata):
	# 	self.use_front_network(True)
	# 	self.init_state()
	# 	forward_mag = .2
	# 	class_num = const.CLASSES['gate']

	# 	# Run for like 2 seconds and see if we see anything
	# 	while((rospy.get_time() - self.current_state_start_time) < 5):
	# 		msg = self.init_joy_msg()
	# 		msg.axes[const.AXES['frontback']] = forward_mag
	# 		detection = self.get_box_of_class(gbl.detections_front, class_num, gbl.current_target)
	# 		if detection != None and detection.score > 0.3:  # If the box is good
	# 			msg = self.align_with_box(detection.box, offsetX=0.7, offsetY=0.5)
	# 			self.publish_joy(msg)
	# 			return 'object_found'
	# 		self.publish_joy(msg)

	# 	return 'object_not_found'

	def log(self):
		rospy.loginfo('Executing state SEARCH_FRONT_BUOY')

