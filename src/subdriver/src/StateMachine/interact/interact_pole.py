#!/usr/bin/env python2

from StateMachine.sub import *
import random

'''
interact_pole.py
Will move around a pole to complete prequalification
Assumes it will start at the distance from the pole that you want to hold when you move around it

Works by strafing to the side while rotating to keep the pole centered in the front camera.
Since rotating also changes the direction we're strafing, the result should a roughly circular orbit around the pole. I think.
It also tries to keep the same distance from the pole by tracking the initial size of the pole (in diagonal distance) and roughly maintaining that (+- 20 percent).
This is likely to be error-prone because of inconsistencies in how much of the pole we can see.

Variables:
init_size
    Number representing the size of the pole when first detected. We use this to keep distance from the pole roughly equal.

'''

# define state interact_pole
class Interact_Pole(Sub):
    #setting to none indicates that we haven't seen it yet
    init_size = None

    def __init__(self):
        smach.State.__init__(self, outcomes=['around_pole','lost_pole'])

    def execute(self, userdata):
        #initialization
        self.init_state()
        self.last_seen = rospy.get_time()
        init_heading = self.get_heading()

        # Start the front network
        self.use_front_network(True)

        #get initial heading
        while not init_heading:
            rospy.sleep(const.SLEEP_TIME)
            init_heading = self.get_heading()

        #keep going until we're within 10 degrees of the opposite of the initial heading
        while (abs(init_heading - self.get_heading()) < 170 and abs(init_heading - self.get_heading()) > 190):
            detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)
            center = self.get_center(detection.box)
            msg = self.init_joy_msg()

            if detection != None:  # If the box is good
                #update values
                if self.init_size == None:
                    self.init_size = self.get_distance_from_box(detection.box)
                self.last_seen = rospy.get_time()
            elif (rospy.get_time - self.last_seen) <= 5:
                #stay still and look around to see if we can pick it back up
                msg.axes[const.AXES['rotate']] = -0.1 * random.randint(-1, 1)
                self.publish_joy(msg)
                rospy.sleep(const.SLEEP_TIME)
                continue
            else: #if last seen more than 5 seconds ago
                return 'Lost_Pole'

            #strafe right
            msg.axes[const.AXES['strafe']] = 0.15

            #keep the pole centered by rotating
            #these are fast, but I'm assuming we want to make sure rotation keeps up so the circle stays tight.
            #Worst-case is probably that we get jumps of fast rotation followed by nothing.
            if center[0] < 0.45:
                msg.axes[const.AXES['rotate']] = 0.1
            elif center[0] > 0.55:
                msg.axes[const.AXES['rotate']] = -0.1

            #maintain distance
            if self.get_distance_from_box(detection.box) > 1.2 * self.init_size:
                msg.axes[const.AXES['frontback']] = -0.2
            elif self.get_distance_from_box(detection.box) < 0.8 * self.init_size:
                msg.axes[const.AXES['frontback']] = 0.2

            #hold depth
            #if we can see the ends of the pole (i.e. the bounding box doesn't end at the edge of the screen), center on it
            if detection.box[1] > 0.1 and detection.box[3] < 0.9:
                if (center[1]) < 0.45:
                    msg.axes[const.AXES['vertical']] = 0.1
                elif center[1] > 0.55:
                    msg.axes[const.AXES['vertical']] = -0.1
            #otherwise no change

            self.publish_joy(msg)
        #end while

        #move out of the pole's way
        #keep strafing without rotating until the pole is to our side
        #exact position is only a guess and will probably need to be modified
        while(True):
            detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)
            center = self.get_center(detection.box)
            if (center[0] > 0.2 and center[0] < 0.8):
                msg = self.init_joy_msg()
                msg.axes[const.AXES['strafe']] = 0.15
                self.publish_joy(msg)
            else:
                break
        
        gbl.current_target = const.CLASSES['start_gate']

        #headed home, motherfuckers
        return 'Around_Pole'
