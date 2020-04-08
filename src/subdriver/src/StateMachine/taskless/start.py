#!/usr/bin/env python2

from StateMachine.sub import *

# define state start
class Start(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_found_gate', 'found_gate'])
        

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

        # Start the front network
        self.use_front_network(True)

        # Initialize joystick message
        curr_msg = self.init_joy_msg()
        curr_msg.axes[const.AXES['frontback']] = 0.5
        curr_msg.axes[const.AXES['vertical']] = -0.7

        gbl.current_target = const.CLASSES['start_gate']

        if(gbl.debug):
            return 'not_found_gate' # Debug Porpoises Only!    

        # Control loop
        while(1):
            self.publish_joy(curr_msg)

            if(rospy.get_time() - self.current_state_start_time) > 2:    
                if self.get_box_of_class(gbl.detections_front, const.CLASSES['start_gate']):
                    return 'found_gate'
                elif (rospy.get_time() - self.current_state_start_time) > 6:
                    return 'not_found_gate'

            rospy.sleep(const.SLEEP_TIME)

