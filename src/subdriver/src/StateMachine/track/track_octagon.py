#!/usr/bin/env python2

from StateMachine.sub import *

'''
Track_Octagon class

substate of Sub state for tracking the octagon/coffin surfacing task
The idea is to try to stay at the depth halfway between them if possible, head straight for them,
and stop once we get close enough we can see the coffin in the bottom camera.
If we don't know the depth halfway between them, we'll guess as well as we can.

Vars:
    self.oct_det - most recent detection of an octagon
    self.coffin_det - most recent detection of a coffin
    self.center_depth - depth of the halfway point between the objects, if we know it
'''


# define state Track_Octagon
class Track_Octagon(Sub):
    oct_det = None
    coffin_det = None
    center_depth = None
    #I thought about also storing the heading to the task, but I'm assuming if we ever knew it
    #we'll be pointed at it when we leave the state, and the Pixhawk will keep us steady after that.
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_octagon','approached_octagon'])

    def execute(self, userdata):
        self.init_state()
        self.last_seen = rospy.get_time()

        # Start the front network
        self.use_front_network(True)
        self.use_bottom_network(True)

        #control loop
        while(1):
            msg = self.init_joy_msg()
            #TODO: Change this to use current target?
            #TODO: change get_boxes_of_classes to return only the highest confidence value of each class
            detections = self.get_boxes_of_classes(gbl.detections_front, ['coffin', 'octagon'])
            
            #TODO: update this to how we actually do it
            if (bottom_camera.can_see_coffin):
                #if we can see the coffin in the bottom camera, we did it - interact_octagon can take over
                return "approached_octagon"
            
            detections = []
            if detections != None:
                #since we're only passing in two classes I'm assuming we can only have two detections
                #these bools are so we know what we have on each loop - maybe not the best way to do this, but it'll work
                #we don't want to lose an object and keep thinking it's the last place we saw it
                oct_found = False
                coffin_found = False
                for det in detections:
                    if det.class_num == const.CLASSES['octagon']:
                        oct_det = det
                        oct_found = True
                    else:
                        coffin_det = det
                        coffin_found = True
                        
                self.last_seen = rospy.get_time()

                #these may be better implemented with the new relative position functions.
                if (oct_found):
                    oct_center = self.get_center(oct_det.box)
                    if (self.get_distance_from_box(oct_det.box) > 0.8):
                        self.is_close = True
                if (coffin_found):
                    coffin_center = self.get_center(coffin_det.box)
                    if (self.get_distance_from_box(coffin_det.box) > 0.8):
                        self.is_close = True
                if (oct_found and coffin_found):
                    #TODO: do this correctly - may have to change get_center
                    #intent is just to treat the two points as rectangle corners to find the center 
                    center_of_both = self.get_center([oct_center[0], coffin_center[0], oct_center[1], coffin_center[1]])
                    
                    #we've already done the size check on the coffin, so now also check whether we're at
                    #the edge of being able to see both objects - if we are, we can probably go straight toward it
                    if (oct_center[1] < 0.2 and coffin_center[1] < 0.8):
                        self.is_close = True

                #Keep Moving Forward
                #   - Walt Disney
                msg.axes[const.AXES['frontback']] = 0.3
  
                #this is basically three different sets of actions depending on what we can see
                #this is the one we want but I don't expect to have it a lot of the time
                if (oct_found and coffin_found):
                    #if we can see both objects, head straight for the center of both of them
                    if center_of_both[0] < 0.45:
                        msg.axes[const.AXES['rotate']] = 0.4
                    elif center_of_both[0] > 0.55:
                        msg.axes[const.AXES['rotate']] = -0.4
    
                    if center_of_both[1] < .45:
                        msg.axes[const.AXES['vertical']] = 0.2
                    elif center_of_both[1] > .55:
                        msg.axes[const.AXES['vertical']] = -0.2
                    else:
                        #store the depth of the center once we're there so we can head to that later instead of guessing
                        #this we don't want to clear out once we lose it - it should be the same no matter where we are
                        self.center_depth = self.get_depth()
                    

                elif (oct_found and not coffin_found):
                    #get heading
                    if oct_center[0] < 0.45:
                        msg.axes[const.AXES['rotate']] = 0.4
                    elif oct_center[0] > 0.55:
                        msg.axes[const.AXES['rotate']] = -0.4

                    #go to depth
                    if self.center_depth:
                        #stay at center depth if we know it
                        if self.get_depth() < self.center_depth - 0.1:
                            msg.axes[const.AXES['vertical']] = 0.2
                        elif self.get_depth > self.center_depth + 0.1:
                            msg.axes[const.AXES['vertical']] = -0.2
                    else:
                        #if we don't know center depth, guess based on the position of the object we can see
                        #we may be able to use the size of the object here too
                        #TODO: use the new bounding box placement functions
                        pass


                elif (oct_found and not coffin_found):
                    #get heading
                    if oct_center[0] < 0.45:
                        msg.axes[const.AXES['rotate']] = 0.4
                    elif oct_center[0] > 0.55:
                        msg.axes[const.AXES['rotate']] = -0.4

                    #go to depth
                    if self.center_depth:
                        #stay at center depth if we know it
                        if self.get_depth() < self.center_depth - 0.1:
                            msg.axes[const.AXES['vertical']] = 0.2
                        elif self.get_depth > self.center_depth + 0.1:
                            msg.axes[const.AXES['vertical']] = -0.2
                    else:
                        #if we don't know center depth, guess based on the position of the object we can see
                        #we may be able to use the size of the object here too
                        #TODO: use the new bounding box placement functions
                        pass
                
                elif (not oct_found and not coffin_found):
                    #either we're close or we lost it
                    if not self.is_close:
                        #if we lost it, we're probably screwed, or failed to detect that we're close.
                        #TODO: implement better smoothing - not sure if that's already done somewhere
                        if (rospy.get_time() - self.last_seen) > 2:
                            msg.axes[const.AXES['frontback']] = 0
                            self.publish_joy(msg)
                            rospy.logwarn("Lost tracking the octagon for more than 2 seconds")
                            if gbl.debug:
                                return "approached_octagon" # DEBUG Purposes Only!
                            else:
                                return "lost_octagon"

                    #if we're close, keep moving forward until we can see the coffin in the bottom camera
                    #since we're already moving straight forward and the pixhawk will keep our depth and heading
                    #stable we don't need to do anything else - we'll catch that we can see the coffin at the beginning of the loop
                    #however, there's only so long we want to go straight
                    #I expect we'll have to kill it before we ever reach this
                    else:
                        if (rospy.get_time() - self.last_seen) > 20:
                            msg.axes[const.AXES['frontback']] = 0
                            self.publish_joy(msg)
                            rospy.logwarn("Got close to the octagon but never saw the coffin")
                            #TODO: may want to have this return something else that flat-out kills the run, 
                            #because we can't come back from this.
                            return "lost_octagon"


            self.publish_joy(msg)

            rospy.sleep(const.SLEEP_TIME)

    def log(self):
      rospy.loginfo('Executing state TRACK_OCTAGON')
