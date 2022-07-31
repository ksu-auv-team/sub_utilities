from StateMachine.sub import *
from StateMachine import PID

# Sole purpose is to test the sub's ability to go rotate right
class Spin(Sub):

	# Gives a list of outcomes from this state
	def __init__(self):
		smach.State.__init__(self,outcomes=['spun_around'])

	def execute(self, userdata):
		self.init_state()
		rotation_magnitude = -.25

		msg = self.init_joy_msg()
		dive_mag = -.15
		msg.axes[const.AXES['vertical']] = dive_mag
        msg.axes[const.AXES['rotate']] = rotation_magnitude
        end_bearing = gbl.init_heading
    # for 5 seconds, go rotate at whatever magnitude
		while((rospy.get_time() - self.current_state_start_time) < 15):
			rospy.loginfo("rotating right")
			self.publish_joy(msg)
			rospy.sleep(const.SLEEP_TIME)
        
        msg2 = self.init_joy_msg()
        msg2.axes[const.AXES['vertical']] = dive_mag
        heading = gbl.init_heading
        pid = PID(0.5, .1, .3, setpoint=heading)
        pid.output_limits = (-0.5, 0.5)

        while (gbl.heading > gbl.init_heading + 1 and gbl.heading < gbl.init_heading - 1)
            rospy.loginfo("resetting bearing")
            msg2.axes[const.AXES['rotate']] = pid(gbl.heading)
            sel.publish_joy(msg2)
            rospy.sleep(const.SLEEP_TIME)
		rospy.loginfo("We should've done the spinny thing")

		# We can absolutely test if we did rotate or not but that's a todo
		# TODO: Cache gbl.state_heading (or was it gbl.heading?) to compare to our new heading
		return 'spun_around'
