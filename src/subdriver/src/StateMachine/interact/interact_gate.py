#!/usr/bin/env python2

from StateMachine.sub import *

# define state interact_gate
class Interact_Gate(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate'])

    def execute(self, userdata):
        self.init_state()

        msg = self.init_joy_msg()
        msg.axes[const.AXES['frontback']] = 0.7


        rospy.loginfo("Charging forward for 5 seconds")

        while rospy.get_time() < (self.current_state_start_time + 5):
            self.publish_joy(msg)
            rospy.sleep(const.SLEEP_TIME)

        gbl.current_target = const.CLASSES['tommy_gun']

        # lazily "Rotates" us to the next objective
        while rospy.get_time() < (self.current_state_start_time + 3):
            msg.axes[const.AXES['frontback']] = 0.0
            msg.axes[const.AXES['rotate']] = 0.5
            self.publish_joy(msg)
            rospy.sleep(const.SLEEP_TIME)

        return 'through_gate'


    def log(self):
        rospy.loginfo('Executing state INTERACT_GATE')