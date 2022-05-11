from StateMachine.sub import *

# Sole purpose is to test the sub's ability to go rotate left
class RotateLeft(Sub):
  
  # Gives a list of outcomes from this state
  def __init__(self):
		smach.State.__init__(self,outcomes=['rotated_left'])
		
	def execute(self, userdata):
		self.init_state()
		rotation_magnitude = .5
		
		msg = self.init_joy_msg()
    
    # for 5 seconds, go rotate at whatever magnitude 
		while(rospy.get_time() - self.current_state_start_time < 5):
			rospy.loginfo("rotating left')
			msg.axes[const.AXES['rotate']] = rotation_magnitude
			self.publish(msg)
			rospy.sleep(const.SLEEP_TIME)
									
		rospy.loginfo("We should've rotated left")
                    
    # We can absolutely test if we did rotate or not but that's a todo
    # TODO: Cache gbl.state_heading (or was it gbl.heading?) to compare to our new heading
		return 'rotated_left'
