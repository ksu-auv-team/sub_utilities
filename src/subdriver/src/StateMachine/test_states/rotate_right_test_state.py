from StateMachine.sub import *

# Sole purpose is to test the sub's ability to go rotate right
class RotateRight(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		smach.State.__init__(self,outcomes=['rotated_right'])

	def execute(self, userdata):
		self.init_state()
		rotation_magnitude = -.5

		msg = self.init_joy_msg()
		dive_mag = -.2
		msg.axes[const.AXES['vertical']] = dive_mag

		msg.axes[const.AXES['rotate']] = rotation_magnitude
    # for 5 seconds, go rotate at whatever magnitude
		while((rospy.get_time() - self.current_state_start_time) < 5):
			rospy.loginfo("rotating right")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've rotated right")

		# We can absolutely test if we did rotate or not but that's a todo
		# TODO: Cache gbl.state_heading (or was it gbl.heading?) to compare to our new heading
		return 'rotated_right'
