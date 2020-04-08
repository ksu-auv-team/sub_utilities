#!/usr/bin/env python
import StateMachine.const
from StateMachine.sub import *
from StateMachine.sub import rospy
from StateMachine.sub import smach

import math

class Launch_Error(Exception):
    '''Base class for exceptions in this module. Indicates a failure to launch.'''
    pass

class Launcher_Ready_Error(Launch_Error):
    '''Indicates that launchers are failing to present as READY.'''
    pass

class Interact_Torpedo(Sub):
    '''Executes interaction state for a task requiring launching torpedoes.
    
    Assumptions:
      - Torpedo launch is mapped to 1 or 2 joystick buttons (left and right
        launchers would need different windage caluclations applied, based on
        distance to target).
      - State is entered from a track_torpedo_target
        such that the desired target is aligned and centered.
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['TORPEDO_SUCCESS',
                                             'TORPEDO_FAILURE'])

    def execute(self, userdata):
        '''Executes the INTERACT_TORPEDO state's primariy action.'''
        #initialization
        self.init_state()
        self.last_seen = rospy.get_time()
        rospy.loginfo('Executing state INTERACT_TORPEDO')

        # Start the front network
        self.use_front_network(True)

        if not self.active_launcher:
            rospy.loginfo('[INTERACT_TORPEDO] - %s' % ('No available launch tubes'))
            return 'torpedo_failed'
        try:
          self.launch(self.active_launcher)
          return 'torpedo_launched'
        except Launch_Error as e:
            #Issues with launchers themselves - failure to ready, failure to
            #fire, etc.
            rospy.loginfo('[INTERACT_TORPEDO] - %s' % (e.message))
            return 'torpedo_failed'
        except Exception as e:  
            #Some other thing is broken, likely this very code.
            rospy.logwarn('[INTERACT_TORPEDO] - %s' % (e.message))
            return 'torpedo_failed'

    def launch(self, launcher):
        '''Launches a torpedo at the target.

        Args:
          launcher: string, id'd launcher, assumed armed and ready to fire.
        Raises:
          Launch_Error: if there is an issue with the launcher(s) preventing
          torpedo launch.
        '''
        try:
            jmsg = self.init_joy_msg()
            jmsg.buttons[const.BUTTONS[const.JOY_MAP[launcher]]]=1
            self.publish(jmsg)
            rospy.sleep(1)
            jmsg.buttons[const.BUTTONS[const.JOY_MAP[launcher]]]=0
            self.publish(jmsg)
            # Activates next tube
            self.set_active_launcher()
        except Exception as e:
            raise Launch_Error(e)






