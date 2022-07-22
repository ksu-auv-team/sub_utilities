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
        """ We will attempt to bump into the Drauger face of the buoy """
        self.init_state()

        # Start the front network
        self.use_front_network(True)

        msg = self.init_joy_msg()

        if not self.get_boxes_of_classes(gbl.detections_front, const.CLASSES['buoy'])[0]:
            return "lost_buoy"


        depth_pid = controllers.PID(s=gbl.depth)
        # strafe_pid = PID(s=)
        # Line up with buoy
        # Since the buoy is spinning, there might be a problem lining up with it. Not sure what to do if that is the case
        # The below code assu-mes it is able to identify the spinning buoy the entire time.
        
        #TODO: implement this method and define this constants
        while self.get_center(self.get_boxes_of_classes(gbl.detections_front, const.CLASSES['buoy'])[0]) != 0:
            self.matchBuoyDepth(msg)
            self.matchBuoyStrafe(msg)
            self.moveCloseToBuoy(msg)

        # At this point, the sub is stationary and facing the Buoy
        return 'locked_onto_buoy'

    def matchBuoyDepth(self, msg):
        rospy.loginfo("matchBuoyDepth: Adjusting depth")
        # controllers.PID depth_pid
        # depth.pid         
        msg.axes[const.AXES['vertical']] = depth_pid.Update(self.get_center(gbl.detections_front[self.findBoxNumber()])[1])
        self.publish_joy(msg)
        rospy.sleep(const.SLEEP_TIME)
        self.publish_joy(msg)

    def matchBuoyStrafe(self, msg):
        rospy.loginfo("matchBuoyStrafe: Adjusting left to right")
        msg.axes[const.AXES['strafe']] = controllers.PID().Update(self.get_center(gbl.boxes[self.findBoxNumber()])[0])
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

        