#!/usr/bin/env python2

from StateMachine.sub import *

# define state interact_gate
class SpinToWin(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate'])

    def execute(self, userdata):
        self.init_state()
        gbl.state_heading = gbl.heading

        msg = self.init_joy_msg()
        msg.axes[const.AXES['frontback']] = 0.6     

        rospy.loginfo('Charging forward for 3 seconds')
        
        while rospy.get_time() < (self.current_state_start_time + 3):
            self.publish(msg)
            rospy.sleep(const.SLEEP_TIME)

        rospy.loginfo('720noscope')
        degrees_spun = 0

        while (degrees_spun < 585):
            msg.axes[const.AXES['frontback']] = 0
            msg.axes[const.AXES['rotate']] = -0.4

        while (abs(self.angle_diff(gbl.state_heading, gbl.init_heading)) < 2):
            msg.axes[const.AXES['frontback']] = 0
            msg = self.center_on_heading(gbl.state_heading, msg)

        
        second_start_time = rospy.get_time()
        rospy.loginfo('Charging forward for three more seconds')
        while rospy.get_time() < (second_start_time + 3):
            self.publish(msg)
            rospy.sleep(const.SLEEP_TIME)

        return 'through_gate'


    def log(self):
        rospy.loginfo('Executing state SPIN2WIN')