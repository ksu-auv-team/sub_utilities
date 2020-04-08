#!/usr/bin/env python

from StateMachine.sub import *

# define state track_pole
class Track_Pole(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_pole','approached_pole'])

    def execute(self, userdata):
        self.init_state()
        self.last_seen = rospy.get_time()

        # Start the front network
        self.use_front_network(True)

        #control loop
        while(1):
            msg = self.init_joy_msg()
            detection = self.get_box_of_class(gbl.detections_front, gbl.current_target)

            if (detection != None) and detection.score > 0.3:  # If the box is good
                self.last_seen = rospy.get_time()
                center = self.get_center(detection.box)

                #move forward
                msg.axes[const.AXES['frontback']] = 0.3
  
                #center horizontally on pole
                if center[0] < 0.45:
                    msg.axes[const.AXES['rotate']] = 0.05
                elif center[0] > 0.55:
                    msg.axes[const.AXES['rotate']] = -0.05
  
                #center vertically on pole
                if center[1] < .45:
                    if self.get_depth() > 0.5:
                        msg.axes[const.AXES['vertical']] = 0.2
                    #else no change - stay at least half a meter below the surface
                elif center[1] > .55:
                    msg.axes[const.AXES['vertical']] = -0.2

                if detection:
                    if self.get_distance(detection.box[0], detection.box[1], detection.box[2], detection.box[3]) > 0.4:
                        self.is_close = True

            if self.is_close:
                self.is_close = False
                return "approached_pole"
            elif (rospy.get_time() - self.last_seen) > 2:
                msg.axes[const.AXES['frontback']] = 0
                self.publish_joy(msg)
                rospy.logwarn("Lost tracking the pole for more than 2 seconds")
                
                if(gbl.debug):
                    return "approached_pole" # DEBUG Purposes Only!
                
                return "lost_pole"

            self.publish_joy(msg)

            rospy.sleep(const.SLEEP_TIME)

    def log(self):
      rospy.loginfo('Executing state TRACK_POLE')