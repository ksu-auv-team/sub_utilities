from StateMachine.sub import *

# Sole purpose is to test the sub's ability to go dive
class DiveDown(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		print("init dive down")
		smach.State.__init__(self,outcomes=['dove'])

	def execute(self, userdata):
		self.init_state()
		vertical_magnitude = -.3
		print("DIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIVE")
		msg = self.init_joy_msg()

		msg.axes[const.AXES['vertical']] = vertical_magnitude
    # for 5 seconds, go rotate at whatever magnitude
		while((rospy.get_time() -(self.current_state_start_time)) < 3):
			rospy.loginfo("DIVE DIVE DIVE")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've went down")

		return 'dove'
