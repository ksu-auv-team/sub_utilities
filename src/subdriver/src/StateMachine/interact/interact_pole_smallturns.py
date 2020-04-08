#!/usr/bin/env python2

from StateMachine.sub import *
import random

'''
interact_pole.py
Will move around a pole to complete prequalification
Assumes it will start at the distance from the pole that you want to hold when you move around it

This version works by turning 90 deg. at a time and keeping the pole on-camera. Once done, we turn back to the original heading and go back to the gate.
It should be slower and uglier and worse than the other method, but it might be easier to modify.

Variables:
init_size
    Number representing the size of the pole when first detected. We use this to keep distance from the pole roughly equal.

'''

# define state interact_pole_smallturns
class Interact_Pole_Small_Turns(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['around_pole','lost_pole'])

    def execute(self, userdata):
        #initialization
        self.init_state()
        self.last_seen = rospy.get_time()
        self.init_heading = self.get_heading()

        # Start the front network
        self.use_front_network(True)

        #keep going until we're within 10 degrees of the opposite of the initial heading
        num_turns = 0
        max_turns = 3
        while (num_turns < max_turns and not (num_turns > 2 and abs(gbl.init_heading - self.get_heading()) < 170 and abs(gbl.init_heading - self.get_heading()) > 190)):
            pole_on_right = False
            pole_on_left = False
            
            detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)
            center = self.get_center(detection.box)
            msg = self.init_joy_msg()

            while (not pole_on_right):
                msg = self.init_joy_msg()
                #hold depth
                detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)

                if detection != None:
                    if center[0] > 0.8:
                        pole_on_right = True

                    #strafe left
                    msg.axes[const.AXES['strafe']] = -0.15

                    #hold depth
                    #if we can see the ends of the pole (i.e. the bounding box doesn't end at the edge of the screen), center on it
                    if detection.box[3] > 0.1 and detection.box[5] < 0.9:
                        if (center[1]) < 0.45:
                            msg.axes[const.AXES['vertical']] = 0.1
                        elif center[1] > 0.55:
                            msg.axes[const.AXES['vertical']] = -0.1
                        #otherwise stay still
                            
                elif rospy.get_time - self.last_seen < 5:
                    #stay still and look around to see if we can pick it back up              
                    msg.axes[const.AXES['rotate']] = -0.1 * random.randint(-1, 1)
                    self.publish_joy(msg)
                    rospy.sleep(const.SLEEP_TIME)
                    continue
                else:
                    return 'Lost_Pole'

            while (not pole_on_left):
                msg = self.init_joy_msg()
                #hold depth
                detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)

                if detection != None:
                    if center[0] < 0.2:
                        pole_on_left= True

                    #rotate right
                    msg.axes[const.AXES['rotate']] = -0.08

                    #hold depth
                    #if we can see the ends of the pole (i.e. the bounding box doesn't end at the edge of the screen), center on it
                    if detection.box[1] > 0.1 and detection.box[3] < 0.9:
                        if (center[1]) < 0.45:
                            msg.axes[const.AXES['vertical']] = 0.1
                        elif center[1] > 0.55:
                            msg.axes[const.AXES['vertical']] = -0.1
                        #otherwise no change

                elif rospy.get_time - self.last_seen < 5:
                    #stay still and look around to see if we can pick it back up
                    msg.axes[const.AXES['rotate']] = -0.1 * random.randint(-1, 1)
                    self.publish_joy(msg)
                    rospy.sleep(const.SLEEP_TIME)
                    continue
                else:
                    return 'Lost_Pole'
            

            self.publish_joy(msg)
            
        #end while

        #Once we've gotten around the pole, aim at the start gate again
        while(True):
            detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)
            center = self.get_center(detection.box)
            while (abs(gbl.init_heading - self.get_heading()) < 170 and abs(gbl.init_heading - self.get_heading()) > 190):
                msg = self.init_joy_msg()
                msg.axes[const.AXES['rotate']] = -0.1
                self.publish_joy(msg)
            else:
                break
        
        gbl.current_target = const.CLASSES['start_gate']

        #headed home, motherfuckers
        return 'Around_Pole'
