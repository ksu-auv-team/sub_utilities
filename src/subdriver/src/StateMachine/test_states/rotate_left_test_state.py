from StateMachine.sub import *
from simple_pid import PID

# Sole purpose is to test the sub's ability to go rotate left
class RotateLeft(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		smach.State.__init__(self,outcomes=['rotated_left'])

	def execute(self, userdata):
		self.init_state()
		rotation_magnitude = .25

		msg = self.init_joy_msg()

		msg.axes[const.AXES['rotate']] = rotation_magnitude
		# for 5 seconds, go rotate at whatever magnitude

		depth = self.get_depth()
		pid = PID(1, 0.1, 0.3, setpoint=(depth-2))
		pid.output_limits = (-0.5, 0.5)
		
		last_time = self.current_state_start_time

		while((rospy.get_time() - self.current_state_start_time) < 5):
			current_time = rospy.get_time()
			dt = rospy.get_time() - last_time

			power = pid(depth)
			depth = self.get_depth()

			msg.axes[const.AXES['vertical']] = power
			
			rospy.loginfo("rotating left")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've rotated left")

    	# We can absolutely test if we did rotate or not but that's a todo
    	# TODO: Cache gbl.state_heading (or was it gbl.heading?) to compare to our new heading
		return 'rotated_left'
