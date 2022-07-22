from StateMachine.sub import *
import controllers
from controllers import PID

class HoldDepth(Sub):

    # Gives a list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self,outcomes=['dead'])

    def execute(self, userdata):
        self.init_state()
        msg = self.init_joy_msg()
        rospy.logdebug("self.current_state_start_depth " + self.current_state_start_depth)
        depth_pid = PID(s=self.current_state_start_depth)

        while(not rospy.is_shutdown()):
            vert_value = depth_pid.Update(gbl.depth)
            print("vert_value " + vert_value)
            msg[const.AXES['vertical']] = vert_value
            self.publish_joy(msg)

        rospy.loginfo("Dying")

        return 'dead'
