#!/usr/bin/env python2

from StateMachine.sub import *

# define state search_front
class Search_Front(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['object_found','object_not_found'])

    def execute(self, userdata):
        self.init_state()
        gbl.state_heading = gbl.heading
        msg = self.init_joy_msg()
        msg.axes[const.AXES['frontback']] = 0.3

        # Start the front network
        self.use_front_network(True)

        while(1):
            self.publish_joy(msg)
            if self.get_box_of_class(gbl.detections_front, gbl.current_target):
                if self.search_frames_seen <= 2:
                    self.search_frames_seen += 1
                else:
                    self.search_frames_seen = 0
                    return "object_found"

            elif (rospy.get_time() - self.current_state_start_time) > 5:
                self.search_frames_seen = 0
                return "object_not_found"

            else:
                self.search_frames_seen = 0
			
            rospy.sleep(const.SLEEP_TIME)
        

