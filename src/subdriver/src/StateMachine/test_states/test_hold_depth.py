from StateMachine.sub import *
from StateMachine import controllers
# from controllers import PID

class HoldDepth(Sub):

    # Gives a list of outcomes from this state
    def __init__(self):
        smach.State.__init__(self,outcomes=['dead'])

    def execute(self, userdata):
        self.init_state()
        msg = self.init_joy_msg()
        rospy.loginfo("self.current_state_start_depth " + str(self.current_state_start_depth))
        depth_pid = controllers.PID(p=.5, i=.1, d= .3, s=self.current_state_start_depth)

        while(not rospy.is_shutdown()):
            vert_value = depth_pid.Update(gbl.depth)
            print('raw-ish vert_val ' + str(vert_value))
            if vert_value <= 0:
                continue
            vert_value = 1 / vert_value
            if (vert_value > .5):
                vert_value = .5
            print("vert_value " + str(vert_value) + ' gbl.depth is ' + str(gbl.depth) + ' GOAL DPETH is ' + str(self.current_state_start_depth))
            msg.axes[const.AXES['vertical']] = vert_value
            self.publish_joy(msg)

        rospy.loginfo("Dying")

        return 'dead'
