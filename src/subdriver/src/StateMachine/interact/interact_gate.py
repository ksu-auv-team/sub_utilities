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
            self.publish(msg)
            rospy.sleep(const.SLEEP_TIME)

        gbl.current_target = const.CLASSES['pole']

        return 'through_gate'


    def log(self):
        rospy.loginfo('Executing state INTERACT_GATE')