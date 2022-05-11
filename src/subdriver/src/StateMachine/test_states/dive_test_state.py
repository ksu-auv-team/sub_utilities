from StateMachine.sub import *

# Sole purpose is to test the sub's ability to go dive
class DiveDown(Sub):
  
  # Gives a list of outcomes from this state
  def __init__(self):
		smach.State.__init__(self,outcomes=['dove'])
		
	def execute(self, userdata):
		self.init_state()
		vertical_magnitude = -.5
		
		msg = self.init_joy_msg()
    
    # for 5 seconds, go rotate at whatever magnitude 
		while(rospy.get_time() - self.current_state_start_time < 5):
			rospy.loginfo("DIVE DIVE DIVE')
			msg.axes[const.AXES['vertical']] = vertical_magnitude
			self.publish(msg)
			rospy.sleep(const.SLEEP_TIME)
									
		rospy.loginfo("We should've went down")

		return 'dove'
