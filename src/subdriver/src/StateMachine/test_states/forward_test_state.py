from StateMachine.sub import *
from simple_pid import PID

# Sole purpose is to test the sub's ability to go forward
class Forward(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		rospy.loginfo(rospy.get_time())
		rospy.loginfo(self.current_state_start_time)
		smach.State.__init__(self,outcomes=['went_forward'])

	def execute(self, userdata):
		rospy.loginfo("GOING FORWARD")
		self.init_state()
		frontback_magnitude = .25

		msg = self.init_joy_msg()

		msg.axes[const.AXES['frontback']] = frontback_magnitude

		depth = self.get_depth()
		pid = PID(1, 0.1, 0.3, setpoint=(self.get_depth()-2))
		pid.output_limits = (-0.5, 0.5)

		last_time = self.current_state_start_time
    	# for 5 seconds, go forward at wahtever magnitude
		while(rospy.get_time() - self.current_state_start_time < 5):
			current_time = rospy.get_time()
			dt = rospy.get_time() - last_time

			power = pid(depth)
			depth = self.get_depth()

			msg.axes[const.AXES['vertical']] = power

			rospy.loginfo("GOING FORWARD")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've gone forward")

    	# I wonder if we can get actual location data from the pixhawk to determine our location
    	# and if we can actually use that to determine if we went forward via software
		return 'went_forward'

