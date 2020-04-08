#!/usr/bin/env python2

from StateMachine.sub import *

# define state start
class Dumb_Start(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup_complete'])
        

    def execute(self, userdata):
        # Set the run start time to the current ros time
        gbl.run_start_time = rospy.get_time() 

        # Initialize the current state
        self.init_state()
        rospy.loginfo("Run Start Time: " + str(gbl.run_start_time))

        # Set the run start depth and heading
        if not gbl.debug:
            while(gbl.depth is None or gbl.heading is None or gbl.init_depth is None or gbl.init_heading is None):
                gbl.init_depth = gbl.depth
                gbl.init_heading = gbl.heading

        # Even though we're techincally not looking for it, we still want to
        # make sure that the rest of the machine knows what we're looking for
        gbl.current_target = const.CLASSES['start_gate']

        # Initialize joystick message
        curr_msg = self.init_joy_msg()
        curr_msg.axes[const.AXES['frontback']] = 0.5 # Forward half-power
        curr_msg.axes[const.AXES['vertical']] = -0.5 # Down half-power

        # Control loop
        while(1):
            self.publish_joy(curr_msg)

            if(rospy.get_time() - self.current_state_start_time) > 5:
                return 'setup_complete'

            rospy.sleep(const.SLEEP_TIME)

