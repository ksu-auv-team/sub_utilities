#!/usr/bin/env python2

from StateMachine.sub import *

# define state surface
class Surface(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['surfaced'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SURFACE')
        gbl.surfacing = True

        msg = self.init_joy_msg()

        while self.get_depth() < 0.2:
            msg.axes[const.AXES['vertical']] = 0.2
            self.publish_joy(msg)
            rospy.sleep(const.SLEEP_TIME)
            if gbl.debug:
                break

        self.thrust_start_time = rospy.get_time()

        #thrust up a bit more to be sure we break the surface
        while(rospy.get_time() < self.thrust_start_time + 1):
            msg.axes[const.AXES['vertical']] = 0.2
            self.publish_joy(msg)
            rospy.sleep(const.SLEEP_TIME)
            if gbl.debug:
                break

        gbl.surfacing = False
        return 'surfaced'
