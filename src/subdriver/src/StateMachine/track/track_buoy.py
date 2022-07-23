#!/usr/bin/env python

from StateMachine.sub import *
from StateMachine import controllers
# from controllers import PID

#TODO: clean up - this needs more work than I'm willing to do right now (while merging stuff)
# add a buoy class group, and reference PID() and msg correctly

# define state track_buoy
class Track_Buoy(Sub):
    """ This state starts with the sub having the buoy in sight.
    In this state, the sub will adjust its depth to match the 3 sided buoy and angle to face it.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['lost_buoy','locked_onto_buoy'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_BUOY')
        self.init_state()

        # Start the front network
        self.use_front_network(True)

        msg = self.init_joy_msg()

        if not self.get_box_of_class(gbl.detections_front, const.CLASSES['tommy_gun'])[0]:
            return "lost_buoy"


        depth_pid = controllers.PID(p=1, i=0.1, d=0.3, s=gbl.depth)
        strafe_pid = controllers.PID(p=1, i=0.1, d=0.3, s=const.CAMERA_FORWARD_CENTER['X'])
        # Line up with buoy

        #TODO: implement this method and define this constants
        while self.get_center(self.get_box_of_class(gbl.detections_front, const.CLASSES['tommy_gun'])[0]) != 0:
            self.matchBuoyDepth(msg, depth_pid)
            self.matchBuoyStrafe(msg, strafe_pid)
            self.moveCloseToBuoy(msg)

        # At this point, the sub is stationary and facing the Buoy
        return 'locked_onto_buoy'

    def matchBuoyDepth(self, msg, depth_pid):
        rospy.loginfo("matchBuoyDepth: Adjusting depth")
        msg.axes[const.AXES['vertical']] = depth_pid.Update(self.get_box_of_class(gbl.detections_front, const.CLASSES['tommy_gun'])[1])
        self.publish_joy(msg)
        rospy.sleep(const.SLEEP_TIME)
        self.publish_joy(msg)

    def matchBuoyStrafe(self, msg, strafe_pid):
        rospy.loginfo("matchBuoyStrafe: Adjusting left to right")
        msg.axes[const.AXES['strafe']] = strafe_pid.Update(self.get_box_of_class(gbl.detections_front, const.CLASSES['tommy_gun'])[1])
        self.publish_joy(msg)
        rospy.sleep(const.SLEEP_TIME)
        # Stop rotating
        msg.axes[const.AXES['strafe']] = 0
        self.publish_joy(msg)

    def moveCloseToBuoy(self, msg):
        rospy.loginfo("moveCloseToBuoy: Moving close to buoy")# While the image width of buoy is less than 0.75
        msg.axes[const.AXES['forward']] = 0.3
        rospy.sleep(const.SLEEP_TIME)
        msg.axes[const.AXES['forward']] = 0
        rospy.loginfo("Done adjusting distance")

