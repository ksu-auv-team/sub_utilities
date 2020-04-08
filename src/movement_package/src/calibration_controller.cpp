#include "calibration_controller.h"

using namespace controller;

CalibrationController::CalibrationController()
    : _calibrationController(HIGH_PWM,LOW_PWM,YAW_CHAN)  
    //PI(int inTopLimit,int inBottomLimit,int inChannel) 
    //THROTTLE_CHAN YAW_CHAN FORWARD_CHAN LATERAL_CHAN
{
    _calibrationSub = _nh.subscribe("controller_calib_data", 10, &CalibrationController::TargetCallback, this); 

}

void CalibrationController::TargetCallback(const std_msgs::Float32MultiArray& msg)
{
    _x=msg.data[0];
    _y=msg.data[1];
    _dist=msg.data[2];
}

void CalibrationController::ProcessChannels()
{
    _mode = 1; //Could be set to either 1 or 2
    
    //void setPID(bool inStatus,int inGoal, int inPose, int inMode);
    //inGoal should be the center of the cam so if 640x480 x_inGoal = 320 y_inGoal = 240
    //inPose is the location of the tracked object the x and y coordinates are treated in the callback function above
    //inMode is for the camera used 1- Forward Facing Camera 2- Downward Facin Camera
    
    _calibrationController.setPID(true, 0, _x, _mode); 

    //Set the mavros commands here
    //We could use the PI::getCommand() function but we'll use the other control commands instead
    //Set the unused channels to MID_PWM (1500)

    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, MID_PWM);
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _calibrationController.yaw_command());
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, MID_PWM);
    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, MID_PWM);
}