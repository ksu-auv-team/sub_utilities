from StateMachine.sub import *
from simple_pid import PID
# Sole purpose is to test the sub's ability to go dive
class SurfaceSub(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		smach.State.__init__(self,outcomes=['gone_up'])

	def execute(self, userdata):
		self.init_state()
		gbl.surfacing = True

		msg = self.init_joy_msg()
		depth = self.get_depth()
		pid = PID(1, 0.1, 0.3, setpoint=0)
		pid.output_limits = (-0.5, 0.5)
		
		last_time = self.current_state_start_time


		# for 5 seconds, go rotate at whatever magnitude
		while((rospy.get_time() - self.current_state_start_time) < 5):
			current_time = rospy.get_time()
			dt = rospy.get_time() - last_time

			power = pid(depth)
			depth = self.get_depth()

			msg.axes[const.AXES['vertical']] = power
			
			rospy.loginfo("YOU RAISE ME UUUUUUUUUPPPPPPPPPPP")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've gone up")

		gbl.surfacing = False
		return 'gone_up'
