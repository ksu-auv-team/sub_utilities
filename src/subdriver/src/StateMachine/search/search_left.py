#!/usr/bin/env python2

from StateMachine.sub import *

# define state search_left
class Search_Left(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['object_found','object_not_found'])

    def execute(self, userdata):
        self.init_state()
        msg = self.init_joy_msg()
        msg.axes[const.AXES['rotate']] = 0.3
        
        # Start the front network
        self.use_front_network(True)

        if(gbl.debug):
            return "object_not_found"

        while(1):
            self.publish_joy(msg)
            if self.get_box_of_class(gbl.detections_front, gbl.current_target):
                if self.search_frames_seen <= 2:
                    self.search_frames_seen += 1
                else:
                    self.search_frames_seen = 0
                    return "object_found"

            elif abs(self.angle_diff(gbl.heading, gbl.state_heading - 45)) < 2:
                self.search_frames_seen = 0
                return "object_not_found"

            else:
                msg = self.center_on_heading(gbl.state_heading - 45, msg)
                self.search_frames_seen = 0

            rospy.sleep(const.SLEEP_TIME)

