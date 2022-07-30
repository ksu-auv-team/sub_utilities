from StateMachine.sub import *
from simple_pid import PID
# Sole purpose is to test the sub's ability to go backwards
class Backwards(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		smach.State.__init__(self,outcomes=['went_backwards'])

	def execute(self, userdata):
		self.init_state()
		frontback_magnitude = -.5

		msg = self.init_joy_msg()

		start_depth = self.get_depth()
		depth = self.get_depth()
		pid = PID(1, 0.1, 0.3, setpoint=start_depth)
		pid.output_limits = (-0.5, 0.5)
		
		last_time = self.current_state_start_time
		# for 5 seconds, go forward at wahtever magnitude
		while((rospy.get_time() - self.current_state_start_time) < 5):
			rospy.loginfo("BACKING UP!")
			msg.axes[const.AXES['frontback']] = frontback_magnitude
			current_time = rospy.get_time()
			dt = rospy.get_time() - last_time

			power = pid(depth)
			depth = self.get_depth()

			msg.axes[const.AXES['vertical']] = power
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've gone backwards")

		# I wonder if we can get actual location data from the pixhawk to determine our location
		# and if we can actually use that to determine if we went forward via software
		return 'went_backwards'
