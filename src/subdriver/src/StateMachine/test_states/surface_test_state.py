from StateMachine.sub import *

# Sole purpose is to test the sub's ability to go dive
class SurfaceSub(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		smach.State.__init__(self,outcomes=['gone_up'])

	def execute(self, userdata):
		self.init_state()
		vertical_magnitude = .5
		gbl.surfacing = True

		msg = self.init_joy_msg()
		msg.axes[const.AXES['vertical']] = vertical_magnitude


		# for 5 seconds, go rotate at whatever magnitude
		while((rospy.get_time() - self.current_state_start_time) < 5):
			rospy.loginfo("YOU RAISE ME UUUUUUUUUPPPPPPPPPPP")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)

		rospy.loginfo("We should've gone up")

		gbl.surfacing = False
		return 'gone_up'
