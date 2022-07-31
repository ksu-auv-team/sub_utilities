from StateMachine.sub import *
import time
from StateMachine import controllers
from StateMachine.depth import DepthControl

# from controllers import PID

class HoldDepth(Sub):

    # Gives a list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self,outcomes=['dead'])

    def execute(self, userdata):
        self.init_state()
        msg = self.init_joy_msg()
        rospy.loginfo("self.current_state_start_depth " + str(self.current_state_start_depth))
        # depth_pid = controllers.PID(p=.5, i=.1, d= .3, s=self.current_state_start_depth)

        depth_control = DepthControl()
        depth_control.run()
        
        while(not rospy.is_shutdown()):
            msg.axes[const.AXES['vertical']] = depth_control.get_power()
            self.publish_joy(msg)
        rospy.loginfo("Dying")

        return 'dead'
