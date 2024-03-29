from StateMachine.sub import *

# Sole purpose is to test the sub's ability to go backwards
class Backwards(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		smach.State.__init__(self,outcomes=['went_backwards'])

	def execute(self, userdata):
		self.init_state()
		frontback_magnitude = -.5

		msg = self.init_joy_msg()
		dive_mag = -.2
		msg.axes[const.AXES['vertical']] = dive_mag

		# for 5 seconds, go forward at wahtever magnitude
		while((rospy.get_time() - self.current_state_start_time) < 5):
			rospy.loginfo("BACKING UP!")
			msg.axes[const.AXES['frontback']] = frontback_magnitude
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've gone backwards")

		# I wonder if we can get actual location data from the pixhawk to determine our location
		# and if we can actually use that to determine if we went forward via software
		return 'went_backwards'
