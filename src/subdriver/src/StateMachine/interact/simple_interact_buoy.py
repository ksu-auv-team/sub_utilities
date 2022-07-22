#!/usr/bin/env python
from StateMachine.gbl import *
from StateMachine.const import *
from StateMachine.sub import *
from StateMachine.controllers import PID

class Simple_Interact_Buoy(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['hit_buoy'])

    def execute(self, userdata):
        """ We will attempt to bump into the BootLegger buoy """
        rospy.loginfo('Executing state INTERACT_BUOY')

        # At this point, the sub is stationary and facing the Buoy
        self.init_state()
        msg = self.init_joy_msg()
        msg.axes[const.AXES['vertical']] = 0.2
        msg.axes[const.AXES['frontback']] = 0.7
        while rospy.get_time() < (self.current_state_start_time + 7):
            self.publish_joy(msg)
            rospy.sleep(const.SLEEP_TIME)
        return 'hit_buoy'
