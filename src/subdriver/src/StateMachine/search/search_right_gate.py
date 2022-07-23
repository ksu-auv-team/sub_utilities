#!/usr/bin/env python2

from  StateMachine.search.search_right import *

# define state search_right_gate
# This inherits from a basic Search_Right class now so we gucci
class Search_Right_Gate(Search_Right):

	# # Gives a list of outcomes from this state
	# def __init__(self):
	# 	smach.State.__init__(self,outcomes=['object_found', 'object_not_found'])

	# def execute(self, userdata):
	# 	self.use_front_network(True)
	# 	self.init_state()
	# 	gbl.state_heading = gbl.heading
	# 	rotate_right_mag = -.2
	# 	class_num = const.CLASSES['gate']

	# 	# Run for like 3 seconds and see if we see anything
	# 	while((rospy.get_time() - self.current_state_start_time) < 2):
	# 		msg = self.init_joy_msg()
	# 		detection = self.get_box_of_class(gbl.detections_front, class_num, gbl.current_target)
	# 		if detection != None and detection.score > 0.3:  # If the box is good
	# 			msg = self.align_with_box(detection.box, offsetX=0.7, offsetY=0.5)
	# 			self.publish_joy(msg)
	# 			return 'object_found'
	# 		msg.axes[const.AXES['rotate']] = rotate_right_mag
	# 		self.publish_joy(msg)


	# 		# Possible PID to rotate back to original heading?
	# 		unitConversion = 0.4 / 360
	# 		kp = 0.8
	# 		ki = 0.1
	# 		kd = 0.5
	# 		integral = 0
	# 		prev_error = 0
	# 		last_time = rospy.get_time()
	# 		while(gbl.heading != gbl.state_heading):
	# 			angle = self.angle_diff(gbl.state_heading, gbl.heading)
	# 			if angle < 0:
	# 				angle += 360

	# 			error = angle * unitConversion
	# 			integral = integral + error * (last_time - rospy.get_time())
	# 			derivative = (error - prev_error) / (last_time - rospy.get_time())

	# 			output = kp*error + ki*integral + kd*derivative
	# 			if output > 0.4:
	# 				output = 0.4
	# 			elif output < -0.4:
	# 				output = -0.4
	# 			msg.axes[const.AXES['rotate']] = output
	# 			self.publish_joy(msg)
	# 			last_time = rospy.get_time()
	# 			prev_error = error
	# 			rospy.sleep(const.SLEEP_TIME)


	# 	return 'object_not_found'

	def log(self):
		rospy.loginfo('Executing state SEARCH_RIGHT_GATE')