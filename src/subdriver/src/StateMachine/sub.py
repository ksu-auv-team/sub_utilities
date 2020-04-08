#!/usr/bin/env python2
'''
sub.py implements a common base class for all states used in the SubDriver
state machine.

As currently implemented, the base class does implicitly depend on state data
stored as global vars in gbl, and system data exposed through rospy API.
'''

#external libraries
import rospy
import smach
import math

import sys, signal

#messages
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import VFR_HUD

from submarine_msgs_srvs.msg import Detections
from classes import Detection

# Global Variables
from StateMachine import gbl
from StateMachine import const

def signal_handler(signal, frame):
    ''' Handles interrupt signals from OS.

    Args:
      signal: signal received from system
      frame: unused? assumed a state capture for diagnostic porpises.
    '''
    rospy.loginfo("\nShutting Down Run...")
    sys.exit(0)

#ROS callbacks
#global to make the linter happy
def vfr_hud_callback(msg): 
    gbl.depth = msg.altitude
    gbl.heading = msg.heading

def bbox_callback_front(msg):
    gbl.detections_front = []
    gbl.num_detections_front = msg.detected[0]

    for i in range(int(gbl.num_detections_front)):
        detection = Detection.Detection(msg.scores[i], msg.boxes[(i*4):(i+1)*4], msg.classes[i])
        gbl.detections_front.append(detection)

def bbox_callback_bottom(msg):
    gbl.detections_bottom = []
    gbl.num_detections_bottom = msg.detected[0]

    for i in range(int(gbl.num_detections_bottom)):
        detection = Detection.Detection(msg.scores[i], msg.boxes[(i*4):(i+1)*4], msg.classes[i])
        gbl.detections_bottom.append(detection)

class Sub(smach.State):
    '''This is our overall 'sub' state. It is the superclass that all the other
    states inherit from.
    
    The idea here being that there are many things that all the states should
    be able to do, but we don't want to re-write them all.
    '''
    def __init__(self):
        '''Initializes a Sub.

        Every state must be initiailized with an __init__ function that defines
        what the outcomes of the state can be. The outcomes determine what
        state is moved to next.
        '''
        smach.State.__init__(self, outcomes=['Finished_Run'])

    def init_state(self):
        '''Initializes a Sub's state using global vars.
        Upon adding the states to the state machine, they run the init_state,
        but we only want them to run it after we have actually started the run.

        Implicit args: gbl.run_start_time, used to check if run is actually
                           started.
                       gbl.depth, current sub depth at start of state.
                       (time), time as returned by rospy.get_time() at start
                           of state.
        '''
        if gbl.run_start_time:
            self.current_state_start_time = rospy.get_time()
            self.current_state_start_depth = gbl.depth
            self.use_front_network(False)
            self.use_bottom_network(False)
            self.log()


    def move_distance(self, distance, direction):
      """ Direcion will be something like: 'strafe', 'vertical', or 'rotate' 
      This function will be fully implemented once data on acceleration is measured in pool tests """
      pass


    def execute(self, userdata):
        '''Executes the behavior defined for a given state.
        Every state requires an 'execute' function. This is the function that automatically
        gets called by SMACH when we transition into the new state. In each of the subclasses 
        that inherit from 'sub' we override the 'execute' function to do that state's speciffic job.

        Args:
            userdata: dict, k-v mapping of data pertinent to state.
        '''
        pass

    def log(self):
      '''Logs to ROS.

      The log function is designed to be overridden in each subclass as a
      catch-all for when you want to log things to ROS.
      '''
      pass

    def init_joy_msg(self):
      '''This initializes a joystick message. We drive our sub by simulating
      joystick commands and sending them to the Pixhawk.

      Returns:
        empty joystick message ready for editing.
      '''
      msg = Joy()
      msg.axes = list(const.DEFAULT_MSG_AXES)
      msg.buttons = list(const.DEFAULT_MSG_BUTTONS)
      return msg

    def get_center(self, box):
        '''Gets the center point of a bounding box.
        

        Args:
          box: Box(type?), box whose center to calculate.
        
        Returns:
          center of a bounding box sent to it
        '''
        print(box)
        return ((box[0] + box[2]) / 2.0 , (box[1] + box[3]) / 2.0)

    def get_distance(self, x1, y1, x2, y2):
        '''Gets distance between two points.
        Args:
            x1,y1,x2,y2: float, scalar coordinates for the two points.

        Returns:
            float, distance between the two points.
        '''
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def get_distance_from_box(self, box):
        '''Gets distance between the corners of a bounding box.

        Args:
            x1,y1,x2,y2: float, scalar coordinates for the two points.

        Returns:
            float, distance between the two points.
        '''
        return math.sqrt((box[4]-box[2])**2 + (box[5]-box[3])**2)

    def align_with_screen(self, box, offsetX = 0.5, offsetY = 0.5):
        """         
        Args:
            box: [top-left x, top-left y, bottom-right x, bottom-right y]

            offsetX: Horizontal offset as a percentage of the screen's width

            offsetY: Vertical offset as a percentage of the screen's height

        Returns:
            a Joystick message to center the sub around the center of a bounding box with
            offsets relative to percentages of the boxes dimensions.
        """
        STRAFE_SPEED = 0.3
        VERTICAL_SPEED = 0.3
        # screenWidth = 1.0
        # screenHeight = 1.0

        center = self.get_center(box)
        msg = self.init_joy_msg()

        # Horizontal
        if(offsetX > center[0]): # Box is to the left of targetX
            msg.axes[const.AXES['strafe']] = -1 * STRAFE_SPEED # Move Right

        elif(offsetX < center[0]): # Box is to the right of targetX
            msg.axes[const.AXES['strafe']] = STRAFE_SPEED # Move Left
        
        
        # Vertical
        if(offsetY > center[1]): # Box is below targetY
            msg.axes[const.AXES['vertical']] = VERTICAL_SPEED # Move Up

        elif(offsetY < center[1]): # Box is above targetY
            msg.axes[const.AXES['vertical']] = -1 * VERTICAL_SPEED # Move Down
        
        return msg


    def align_with_box(self, box, offsetX = 0.5, offsetY = 0.5):
        """ 
        Args:
            box: [top-left x, top-left y, bottom-right x, bottom-right y]

            offsetX: Horizontal offset as a percentage of the box's width

            offsetY: Vertical offset as a percentage of the box's height

        Returns:
            a Joystick message to center the sub around the center of a bounding box with
            offsets relative to percentages of the boxes dimensions.
        """
        STRAFE_SPEED = 0.3
        VERTICAL_SPEED = 0.3


        boxWidth = box[2] - box[0] # box[right] - box[left]
        boxHeight = box[3] - box[1] #box[bottom] - box[top]
        #center = self.get_center(box)
        msg = self.init_joy_msg()
        
        relativeX = box[0] + offsetX * boxWidth # Target x position relative to current
        relativeY = box[1] + offsetY * boxHeight # Target y position relative to current

        # Horizontal
        if(relativeX != 0.5):

            if(relativeX > 0.5): # Target Position is to the right
                msg.axes[const.AXES['strafe']] = -1 * STRAFE_SPEED
            elif(relativeX < 0.5): # Target Position is to the left
                msg.axes[const.AXES['strafe']] = STRAFE_SPEED
        
        # Vertical
        if(relativeY != 0.5):

            if(relativeY > 0.5): # Target Position is to the below
                msg.axes[const.AXES['vertical']] = -1 * VERTICAL_SPEED
            elif(relativeY < 0.5): # Target Position is to the above
                msg.axes[const.AXES['vertical']] = VERTICAL_SPEED
        
        return msg

    def center_on_heading(self, target, msg = [], min_thrust=0.1, max_thrust=0.4):
        '''
        Center the sub on the target heading target.
        Will end up bobbing back and forth around the center, but it should work well enough

        Args:
            target - heading in degrees (0-360)
            msg - joy_msg to modify (default is init_joy_msg)
            min_thrust - minumum thrust amount. Must be positive and less than max_thrust.
            max_thrust - maximum thrust amount. Must be positive and greater than min_thrust
        Returns:
            msg, modified to point the sub to target
        '''

        factor = 0.005
        if not msg:
            msg = self.init_joy_msg()

        diff = self.angle_diff(gbl.heading, target)
        
        val = diff * factor

        if val > 0:
            val = max(val, min_thrust)
            val = min(val, max_thrust)
        elif val < 0:
            val = min(val, max_thrust * -1)
            val = max(val, min_thrust * -1)

        msg.axes[const.AXES['rotate']] = val * -1

        return msg



    def get_depth(self):
        if gbl.debug:
            rospy.loginfo("Depth is none, but we are debugging, returning 0.0")
            return 0.0
        elif (gbl.depth is None or gbl.init_depth is None):
            rospy.logerr("init depth or current depth is NONE")
            return None
        return gbl.depth - gbl.init_depth

    def get_heading(self):
        return gbl.heading

    def use_front_network(self, do_use_network):
       self.front_network_enable_pub.publish(do_use_network)

    def use_bottom_network(self, do_use_network):
        self.bottom_network_enable_pub.publish(do_use_network)

    def get_box_of_class(self, detections, class_num, threshold = 0.30):
        '''
        Takes a list of detections from the neural network and returns the detection in which
        an object of class class_num was found with the highest confidence.

        Parameters:
            detections - list of detection objects
            class_num - ID of class to detect
            threshold - Probability confidence needed to ID an object. Should be between 0 and 1.

        Returns:
            None or a bounding box (list)
        '''

        if detections == []:
            rospy.loginfo('No detections in image at time: ' + str(rospy.get_time()))
            return None

        found = None
        max_prob = 0.0
        for detection in detections:
            if detection.class_id == class_num and detection.score > max_prob:
                found = detection
                max_prob = detection.score

        if found:
            rospy.loginfo('class: %s\tconf: %s', str(found.class_id), str(found.score))

        #ignore ghosts
        if max_prob > threshold:
            return found
        else:
            return None

    def get_boxes_of_classes(self, detections, classes, threshold = 0.30):
        '''
        More versatile, but slower version of get_box_of_class that takes multiple classes and returns
        a list of all detections in which any of them are found with confidence >= threshold.

        Parameters:
            detections - list of detection objects (implemented as lists)
            classes - List of IDs of classes to detect
            confidence - Confidence threshold to ID an object. Should be between 0 and 1.

        Returns:
            List of detections. Empty list if none found.
        '''

        found_detections = []

        if detections == []:
            rospy.sleep(1)
            rospy.loginfo('No boxes in image at time: ' + str(rospy.get_time()))
            return None

        rospy.loginfo('Detections:\n')
        for class_num in classes:
            best_confidence = threshold
            best_det = None
            for det in detections:
                if det.class_id == class_num and det.score > best_confidence:
                    best_det = det
                    best_confidence = det.score

            if (best_det):
                found_detections.append(best_det)
                rospy.loginfo('\tclass: %s\tconf: %s', str(det.class_id), str(det.score))   
        
        return found_detections     


        
    def set_active_launcher(self):
        '''Changes active launcher. 
            Assumes that the first launcher will be LAUNCHER_LEFT and that the function will be called every time a launcher is used to select the next one.
        '''
        if self.active_launcher == 'LAUNCHER_LEFT':
            self.active_launcher = 'LAUNCHER_RIGHT'
            self.active_launcher_offset = const.LAUNCHER_RIGHT_OFFSET
            return
        else:
            self.active_launcher = None
            self.active_launcher_offset = None
            return

    def publish(self, msg):
        '''
        Publish a joy message with joy_pub.
        This exists to let us modify all messages in one place before they're published,
        e.g. to stop all forward motion, speed up all rotation, or mirror all horizontal motion.
        '''

        #modifiers go here

        #mirror run
        if (const.FLIP_RUN):
            msg.axes["strafe"] = msg.axes["strafe"] * -1
            msg.axes["rotate"] = msg.axes["rotate"] * -1

        #publish message
        self.joy_pub.publish(msg)

    def angle_diff(self, source, target):
        '''
        Returns the difference between the two angles. Wraps around so that, e.g.,
        angle_diff(20, 330) returns -50 and angle_diff(330, 20) returns 50. angle_diff(90, 180)
        returns 90 and angle_diff(180, 90) returns -90.
        i.e. the number it returns is always the shorter way around and will be negative if necessary 
        (if target is counterclockwise from/smaller than source)
        '''
        if gbl.debug:
            return 0
        
        elif target is None or source is None:
            return None

        diff = target - source
        return ((diff + 180) % 360) - 180


    def publish_joy(self, msg):
        '''
        Accepts a ROS joystick message as a parameter, modifies it, then republishes it.
        Intended to be used to scale speed globally, among other potential uses.

        Args:
            msg: ROS joystick message
        '''

        #msg['frontback'] = msg['frontback'] * 0.9

        if gbl.debug:
            pass

        #limit depth
        elif (not gbl.surfacing and self.get_depth() < 0.5 and msg.axes[const.AXES['vertical']] > 0):
            msg.axes[const.AXES['vertical']] = 0

        #changes go here
        self.joy_pub.publish(msg) 

    # These get set at the start of each state, allowing the user to call them as needed
    current_state_start_time = None
    current_state_start_depth = None

    signal.signal(signal.SIGINT, signal_handler)

    search_frames_seen = 0
    last_seen = None

    is_close = False
    init_distance = 0

    joy_pub = rospy.Publisher('joy', Joy, queue_size=2)
    front_network_enable_pub = rospy.Publisher('enable_front_network', Bool, queue_size=1)
    bottom_network_enable_pub = rospy.Publisher('enable_bottom_network', Bool, queue_size=1)
    front_network_sub = rospy.Subscriber('front_network_output', Detections, bbox_callback_front, queue_size=1)
    bottom_network_sub = rospy.Subscriber('bottom_network_output', Detections, bbox_callback_bottom, queue_size=1)
    vfr_hud_sub = rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, vfr_hud_callback) #provides depth and heading

    active_launcher = 'LAUNCHER_LEFT'
    active_launcher_offset = const.LAUNCHER_LEFT_OFFSET

