from StateMachine.sub import *
from StateMachine import PID

# Sole purpose is to test the sub's ability to go forward
class Forward(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		rospy.loginfo(rospy.get_time())
		rospy.loginfo(self.current_state_start_time)
		smach.State.__init__(self,outcomes=['went_forward_zoom'])

	def execute(self, userdata):
		rospy.loginfo("GOING FORWARD")
		self.init_state()
		frontback_magnitude = .5

		msg = self.init_joy_msg()
		dive_mag = -.15

        bearing = gbl.init_heading
        pid = PID(0.5, .1, .3, setpoint=bearing)
        pid.output_limits = (-0.5, 0.5)


		msg.axes[const.AXES['vertical']] = dive_mag
		msg.axes[const.AXES['frontback']] = frontback_magnitude
    	# for 5 seconds, go forward at wahtever magnitude
		while(rospy.get_time() - self.current_state_start_time < 1):
			power = pid(gbl.heading)
            msg.axes[const.AXES['rotate']] = power
            
            rospy.loginfo("GOING FORWARD")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)
		rospy.loginfo("We should've gone forward")

    	# I wonder if we can get actual location data from the pixhawk to determine our location
    	# and if we can actually use that to determine if we went forward via software
		return 'went_forward_zoom'

