#include "manual_controller.h"
using namespace controller;

ManualController::ManualController(int lateral, int throttle, int forward, int yaw, int trigger)
{

    _lateral = lateral;
    _throttle = throttle;
    _forward = forward;
    _yaw = yaw;
    _trigger = trigger;
    _joyStickSub = _n.subscribe("joy", 10, &ManualController::JoyStickCallback, this);

    _n.setParam("joy_node/dev", "/dev/input/js0");

    _joyMsg.axes = {-0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    _joyMsg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    this->Arm();

    this->Sequencing();

    this->Disarm();
    _manualArmed = false;

    MavrosCommunicator->SetModeAltHold();
}

void ManualController::JoyStickCallback(const sensor_msgs::Joy& msg)
{
    _lastMsgRecieved = ros::Time::now().toSec();
    _joyMsg = msg;
}

void ManualController::SafeArm()
{
    int messageTime(ros::Time::now().toSec() - _lastMsgRecieved);
    if (!_manualArmed){
        auto trigger_val = _joyMsg.axes[_trigger];
        if (trigger_val < -0.5 && messageTime < 60)//trigger pressed
        {
            this->Arm();
            _manualArmed = true;
        }
    }
    else //armed
    {
    auto trigger_val = _joyMsg.axes[_trigger];
    if (trigger_val >= -0.5) // trigger not pressed
     {
        this->Disarm();
        _manualArmed = false;
    }
    else if(messageTime > 60)
    {
        this->Disarm();
        _manualArmed = false;
    }
    }
}

void ManualController::ProcessChannels()
{
    SafeArm();//trigger-arm

    // VALIDATE THAT THESE ARE STILL GOOD, had
    float lateral_input  = _joyMsg.axes[_lateral];
    float forward_input  = _joyMsg.axes[_forward];
    float throttle_input = _joyMsg.axes[_throttle];
    float yaw_input      = _joyMsg.axes[_yaw];
    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, lateral_input*-500 + MID_PWM);//right stick left-right
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, forward_input*500 + MID_PWM);//right stick up-down
    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, throttle_input*500 + MID_PWM);//left stick up-down
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN,      yaw_input*-500 + MID_PWM);//left stick left-right
}
