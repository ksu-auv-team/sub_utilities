from StateMachine.sub import *
import math

# Sole purpose is to test the sub's ability to strafe left
class StrafeLeft(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		smach.State.__init__(self,outcomes=['strafed_left'])

	def execute(self, userdata):
		self.init_state()
		strafe_magnitude = .25

		msg = self.init_joy_msg()
		dive_mag = -.1
		msg.axes[const.AXES['vertical']] = dive_mag

		msg.axes[const.AXES['strafe']] =  strafe_magnitude
		while((rospy.get_time() - self.current_state_start_time) < 5):
			rospy.loginfo("strafing left")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've strafed left")
		return 'strafed_left'

