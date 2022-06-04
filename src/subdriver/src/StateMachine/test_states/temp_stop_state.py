from StateMachine.sub import *
import math

# Sole purpose is to test the sub's ability to strafe left
class StopState(Sub):

# Gives a list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self,outcomes=['stopped'])

    def execute(self, userdata):
        self.init_state()
        rospy.loginfo("STOPPING JUST IN CASE")
        msg = self.init_joy_msg()
        msg.axes[const.AXES['vertical']] = 0
        msg.axes[const.AXES['strafe']]
        msg.axes[const.AXES['strafe']] = 0
        self.publish_joy(msg)

        return 'stopped'
