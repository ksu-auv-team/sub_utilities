#!/usr/bin/env python3

from StateMachine.sub import *
import math


#Sub will move forward while rotating in place
class SpinForward(Sub):

    #This gives the list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self, outcomes=['through_gate'])


    #Takes the difference in angles and finds the frontback component
    def calc_frontback(self, angle, magnitude):
        frontback = 0.0
        frontback = math.cos(math.radians(angle)) * magnitude
        return frontback

    #Takes the difference in angles and finds the strafe component
    def calc_strafe(self, angle, magnitude):
        strafe = 0.0
        strafe = math.sin(math.radians(angle)) * magnitude
        return strafe
    


    def execute(self, userdata):
        self.init_state()

        gbl.state_heading = gbl.heading
        msg = self.init_joy_msg()

        #Used to calculate FB/strafe, should be how fast the sub goes in the calculated directions
        magnitude = .4

        #Rotate speed
        msg.axes[const.AXES['rotate']] = -.4

        while(rospy.get_time() - self.current_state_start_time < 10):
            #calculates the difference between the init heading at the start of the state
            # and the current heading as the sub rotates
            angle = self.angle_diff(gbl.state_heading, gbl.heading)
            if angle < 0:
                angle += 360

            rospy.loginfo("Angle Difference " + str(angle))

            frontback = self.calc_frontback(angle, magnitude)
            strafe = self.calc_strafe(angle, magnitude)
            msg.axes[const.AXES['strafe']] = strafe
            msg.axes[const.AXES['frontback']] = frontback
            
            self.publish_joy(msg)

            rospy.sleep(const.SLEEP_TIME)
            
    

        return 'through_gate'
