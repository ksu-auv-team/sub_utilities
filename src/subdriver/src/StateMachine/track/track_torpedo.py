#! /usr/bin/env python

#Explicit imports help understand what comes from where (and makes linter happy)
# from StateMachine.const import *
from StateMachine.controllers import PID
from StateMachine import const
from StateMachine.sub import Sub
from StateMachine.sub import rospy
from StateMachine.sub import smach
from StateMachine.sub import gbl

import math

class Track_Torpedo(Sub):
    '''Tracker for torpedo targets.

    Maintains sub position relative to torpedo target,
    applying necessary windage adjustments for the ready tube.
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_lost','target_locked','hardware_failure'])

    def execute(self, userdata):
        self.init_state()
        self.last_seen = rospy.get_time()

        # Start the front network
        self.use_front_network(True)

        # Fail Fast
        if not self.active_launcher:
            print('[TRACK_TORPEDO] - %s' % ('No available launch tubes'))
            return 'hardware_failure'
        self.init_state()
        self.last_seen = rospy.get_time()
        #TODO: Tune these controllers
        x_pid = PID(s=const.CAMERA_FORWARD_CENTER['X'] + self.active_launcher_offset['WINDAGE_OFFSET'])
        z_pid = PID(s=const.CAMERA_FORWARD_CENTER['Z'] + self.active_launcher_offset['ELEVATION_OFFSET'])

        while(1):
            jmsg = self.init_joy_msg()
            detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)

            if (detection is not None) and detection.box[1] > 0.3:
                x,z = self.get_center(detection.box)
            if (abs(const.CAMERA_FORWARD_CENTER['x']-x) < 5 and abs(const.CAMERA_FORWARD_CENTER['z']-z) < 5):
                return 'target_locked'
            jmsg.axes[const.AXES['strafe']] = x_pid.Update(x)
            jmsg.axes[const.AXES['vertical']] = z_pid.Update(z)
            self.publish(jmsg)


    def log(self):
        rospy.loginfo('Executing state TRACK_TORPEDO')
