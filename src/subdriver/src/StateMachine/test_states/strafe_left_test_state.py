from StateMachine.sub import *
import math
from simple_pid import PID
# Sole purpose is to test the sub's ability to strafe left
class StrafeLeft(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		smach.State.__init__(self,outcomes=['strafed_left'])

	def execute(self, userdata):
		self.init_state()
		strafe_magnitude = .25

		msg = self.init_joy_msg()

		msg.axes[const.AXES['strafe']] =  strafe_magnitude

		start_depth = self.get_depth()
		depth = self.get_depth()
		pid = PID(1, 0.1, 0.3, setpoint=start_depth)
		pid.output_limits = (-0.5, 0.5)

		last_time = self.current_state_start_time
		while((rospy.get_time() - self.current_state_start_time) < 5):
			
			current_time = rospy.get_time()
			dt = rospy.get_time() - last_time

			power = pid(depth)
			depth = self.get_depth()

			msg.axes[const.AXES['vertical']] = power

			rospy.loginfo("strafing left")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've strafed left")
		return 'strafed_left'

