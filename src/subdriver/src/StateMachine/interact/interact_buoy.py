#!/usr/bin/env python
from StateMachine.gbl import *
from StateMachine.const import *
from StateMachine.sub import *
from StateMachine.controllers import PID
import math
from enum import Enum

# define state interact_buoy

# Known Information:
    # Buoy Dimensions:  24 in. wide by 48 in. tall
    # Buoy will rotate 1-5 RPM
    # There is a two sided buoy with a Jiangshi on it. Do not target it
    # There is a three sided  buoy with draugr, vetalas, and aswang on it.
        # We choose which one we will target and we will get 600 pts for hitting the one we chose
        # And 300 pts if we hit one of the other two.
    # Target Face of buoy: Since it is arbitrary, we will be targetting the draugr

# Calculated Information:
    # When to move forward
    # Rotation speed of buoy 
    # Current face on buoy
    # Order of faces (determined by which direction that the buoy is spinnning)

# Needed information:

#Python execute_withState
#logitech camera 1920 x 1080p camera:
    # Logitech web cam C930e


class Interact_Buoy(Sub):
    def __init__(self):
        smach.State.__init__(self, outcomes=['clear_of_buoy'])
        self.rotationOrder = -1
        self.targetFace = BuoyFaces.draugr
        self.maxAcceleration = 20
    def execute(self, userdata):
        """ We will attempt to bump into the draugr face of the buoy """
        rospy.loginfo('Executing state INTERACT_BUOY')
        # At this point, the sub is stationary and facing the Buoy
        self.init_state()
        msg = self.init_joy_msg()
        
        # Determine Ideal move speed
        
        
        
        buoyRotationSpeed = self.determineRotationSpeed()
        rospy.loginfo("Found Rotation Speed.")

        # T = rotations/minute * 2pi rad/1 rotation * 60 seconds/1 minute
        period = buoyRotationSpeed * 2 * math.pi * 60

        # Move towards buoy
        rospy.loginfo("Preparing to move forward.")
        while self.findFace() != BuoyFaces.draugr:
            rospy.sleep(period/6)
            startTime=rospy.Time.now()
            rospy.loginfo("Moving forward for 10 seconds")
        
        while rospy.Time.now() < startTime + 10000: # Move for 10 seconds
            msg.axes[const.AXES['forward']] = 1
            self.publish(msg)
            rospy.sleep(const.SLEEP_TIME)
        
        msg.axes[const.AXES['forward']] = 0
        rospy.loginfo("Done moving")
        gbl.current_target = None
        return 'clear_of_buoy'

    #TODO: replace with sub getBoxOfClasses
    def findBox(self):
        # Returns the index in gbl.boxes that the buoy is in
        for i in range(0, len(boxes)):
            if(boxes[i][1] == BuoyFaces.draugr or boxes[i][1] == BuoyFaces.aswang or boxes[i][1] == BuoyFaces.vetalas):
                return i[1]
        return -1 # Face was not found

    #TODO: either remove if getBoxOfClasses makes it redundant or
    # make more sophisticated (use confidence values)
    def findFace(self):
        # returns the face of the buoy that is currently visible
        box = self.findBox()
        if(box<0):
            return -1
        return boxes[box][1]

    def determineRotationSpeed(self):
        """ returns rotation speed in RPM,
        or Returns -1 if unable to find order, but that doesn't necessarily mean that the 
        buoy is lost."""
        # Find first face
        # Skip the first face found, because it could be found halfway through the time spent on that face.
        if(self.buoyIsLost()):
            rospy.loginfo("Buoy lost in def determineRotationSpeed(self) at time: ", rospy.Time.now())
            return -1
        startingFace = self.findFace()
        
        while(self.findFace() == startingFace):
            continue
        
        if(self.buoyIsLost()):
            rospy.loginfo("Buoy lost in def determineRotationSpeed(self) at time: ", rospy.Time.now())
            return -1
        
        firstFaceFoundTime = rospy.Time.now()
        
        firstFace = self.findFace()
        # Find second face
        while(self.findFace() == firstFace):
            continue
        
        if(self.buoyIsLost()):
            rospy.loginfo("Buoy lost in def determineRotationSpeed(self) at time: ", rospy.Time.now())
            return -1
        secondFaceFoundTime = rospy.Time.now()
        secondFace = self.findFace()
        
        if(firstFace == secondFace):
            return -1
        # Determine Order by waiting for a face to appear, then waiting until the next face appears
        
        if(firstFace == BuoyFaces.draugr): #draugr
            
            if(secondFace == BuoyFaces.aswang):
                self.rotationOrder = BuoyRotationOrder.DAV
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
        
            elif(secondFace == BuoyFaces.vetalas):
                self.rotationOrder = BuoyRotationOrder.VAD
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
        
            else:
                return -1
        
        elif(firstFace == BuoyFaces.aswang): #aswang
            
            if(secondFace == BuoyFaces.draugr):
                self.rotationOrder = BuoyRotationOrder.VAD
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            
            elif(secondFace == BuoyFaces.vetalas):
                self.rotationOrder = BuoyRotationOrder.DAV
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            
            else:
                return -1
        
        elif(firstFace == BuoyFaces.vetalas): #vetalas
            if(secondFace == BuoyFaces.aswang):
                self.rotationOrder = BuoyRotationOrder.VAD
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            elif(secondFace == BuoyFaces.draugr):
                self.rotationOrder = BuoyRotationOrder.DAV
                return (secondFaceFoundTime - firstFaceFoundTime) * 3 * 60
            else:
                return -1
        else:
            return -1

    def buoyIsLost(self):
        # Waits a while until it sees the buoy. Returns true if buoy is lost.
        """ Wait up to 30 seconds to see if the buoy any of the monsters on the 3 sided buoy are 
        found. If none are found, then the buoy is assumed to be lost. """
        start = rospy.Time.now()
        while(rospy.Time.now()<start+30):
            if(self.findFace()>=0):
                return False
        return True
           
    def nextFace(self, face):
        # Given a buoy face, method will return the next face in the order
        if(face == BuoyFaces.draugr):
            if(self.rotationOrder == BuoyRotationOrder.DAV):
                return BuoyFaces.aswang
            else:
                return BuoyFaces.vetalas
        
        elif(face == BuoyFaces.aswang):
            if(self.rotationOrder == BuoyRotationOrder.DAV):
                return BuoyFaces.vetalas
            else:
                return BuoyFaces.draugr
        
        elif(face == BuoyFaces.vetalas):
            if(self.rotationOrder == BuoyRotationOrder.DAV):
                return BuoyFaces.draugr
            else:
                return BuoyFaces.aswang

    def getThirdAtTime(self, period, startTime, currentTime):
        # Finds out what face will be showing at a certain time
        # The reason for a startime parameter is because the state does not start at time 0.
        # T = 2pi/w
        # Formula for a sinusodal function: y = ASin(wt + phaseShift)
        w = 2*math.pi/period
        x =  math.cos(w(currentTime-startTime))
        y = math.sin(w(currentTime-startTime))
        if(-1/2 < x and x < 1): # 1st or 3rd third
            if(y>0):
                return 1
            else:
                return 3
        else: # 2nd Third
            return 2


class BuoyFaces(Enum): # Numbers will need to be changed to a proper class_id
    draugr = 0
    aswang = 1
    vetalas = 2

class BuoyRotationOrder(Enum):
    """Enumeration for if the rotation order is:
     draugr, aswang, vetalas (DAV)
     or vetalas, aswang, draugr (VAD)
    """
    DAV = 0
    VAD = 1
    Unknown = 3





