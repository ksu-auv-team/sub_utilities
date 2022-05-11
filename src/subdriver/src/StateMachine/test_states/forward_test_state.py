from StateMachine.sub import *

# Sole purpose is to test the sub's ability to go forward
class Forward(Sub):
  
  # Gives a list of outcomes from this state
  def __init__(self):
		smach.State.__init__(self,outcomes=['went_forward'])
		
	def execute(self, userdata):
		self.init_state()
		frontback_magnitude = .5
		
		msg = self.init_joy_msg()
    
    # for 5 seconds, go forward at wahtever magnitude 
		while(rospy.get_time() - self.current_state_start_time < 5):
			rospy.loginfo("GOING FORWARD')
			msg.axes[const.AXES['frontback']] = frontback_magnitude
			self.publish(msg)
			rospy.sleep(const.SLEEP_TIME)
									
		rospy.loginfo("We should've gone forward")
                    
    # I wonder if we can get actual location data from the pixhawk to determine our location
    # and if we can actually use that to determine if we went forward via software
		return 'went_forward'
					
