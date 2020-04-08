#!/usr/bin/env python2

from StateMachine.sub import *

# define state track_gate
class Track_Gate(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_gate','approached_gate'])

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

                '''
                if center[0] < 0.45:
                    msg.axes[const.AXES['rotate']] = 0.2
                elif center[0] > 0.55:
                    msg.axes[const.AXES['rotate']] = -0.2
  
                if center[1] < .45:
                    if self.get_depth() > 0.5:
                        msg.axes[const.AXES['vertical']] =  0.2
                    #else no change - don't want to go less than half a meter below the surface
                elif center[1] > .55:
                    msg.axes[const.AXES['vertical']] = -0.2
                '''
                
                msg = self.align_with_box(detection.box, offsetX=0.7, offsetY=0.5)
                
                msg.axes[const.AXES['frontback']] = 0.3
                
                
                if detection:
                    if self.get_distance(detection.box[0], detection.box[1], detection.box[2], detection.box[3]) > 0.8:
                        self.is_close = True

                #stay within 20 degrees of run initial heading
                #we'll always start pointed at the gate, so we'll never want more than that unless something goes wrong.
                if self.angle_diff(gbl.heading, gbl.init_heading) > 20:
                    #go left
                    msg.axes[const.AXES['rotate']] = 0.2
                elif self.angle_diff(gbl.heading, gbl.init_heading) < -20:
                    #go right
                    msg.axes[const.AXES['rotate']] = -0.2


            if self.is_close:
                self.is_close = False
                return "approached_gate"
            elif (rospy.get_time() - self.last_seen) > 2:
                msg.axes[const.AXES['frontback']] = 0
                self.publish_joy(msg)
                rospy.logwarn("Lost tracking the gate for more than 2 seconds")
                
                if(gbl.debug):
                    return "approached_gate" # DEBUG Purposes Only!
                
                return "lost_gate"

            self.publish_joy(msg)

            rospy.sleep(const.SLEEP_TIME)

    def log(self):
        rospy.loginfo('Executing state TRACK_GATE')
