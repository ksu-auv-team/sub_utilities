from StateMachine.sub import *
import time
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
        # depth_pid = controllers.PID(p=.5, i=.1, d= .3, s=self.current_state_start_depth)
        depth_state = 0
        time_waited = 0
        while(not rospy.is_shutdown()):
            # vert_value = depth_pid.Update(self.get_depth())
            # print('raw-ish vert_val ' + str(vert_value) + " gbl depth " + str(self.get_depth()))
            # try:
            #     vert_value = 1 / vert_value
            # except:
            #     vert_value = 0
            # if (vert_value > .2):
            #     vert_value = .2
            # elif (vert_value < -.2):
            #     vert_value = -.2
            # print("vert_value " + str(vert_value) + ' gbl.depth is ' + str(gbl.depth) + ' GOAL DPETH is ' + str(self.current_state_start_depth))
            if (depth_state == 0):
                msg.axes[const.AXES['vertical']] = -.15
                if (time_waited == const.WAIT_TIME):
                    depth_state = 1
                    time_waited = 0
            elif (depth_state == 1):
                msg.axes[const.AXES['vertical']] = 0
                if (time_waited == const.WAIT_TIME):
                    depth_state = 0
                    time_waited = 0
            self.publish_joy(msg)
            time_waited += const.SLEEP_TIME
            rospy.sleep(const.SLEEP_TIME)
            

        rospy.loginfo("Dying")

        return 'dead'
