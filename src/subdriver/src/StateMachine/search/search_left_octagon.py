#!/usr/bin/env python2

from StateMachine.sub import *

'''
Implements search turning left for the octagon/surfacing tasks
Different from the other searches because it needs to look for
multiple objects, the coffin and the octagon
'''

# define state search_left
class Search_Left_Octagon(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['object_found','object_not_found'])

    def execute(self, userdata):
        self.init_state()
        msg = self.init_joy_msg()
        msg.axes[const.AXES['rotate']] = -.3

        #return 'object_not_found' # Debug purposes only!

        while(1):
            self.joy_pub.publish(msg)
            #will need to change this to multiple targets
            if self.get_box_of_class(gbl.detections_bottom, gbl.current_target):
                if self.search_frames_seen <= 2:
                    self.search_frames_seen += 1
                else:
                    return "object_found"

            elif self.angle_diff(gbl.heading, gbl.state_heading - 45) <= 0:
                self.search_frames_seen = 0
                return "object_not_found"

            else:
                self.search_frames_seen = 0

            rospy.sleep(const.SLEEP_TIME)
