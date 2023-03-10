#include "manual_controller.h"
#include <unordered_map>
#include <string>

using namespace controller;

ManualController::ManualController()
{
    _joyStickSub = _n.subscribe("joy", 10, &ManualController::JoyStickCallback, this);

    _n.setParam("joy_node/dev", "/dev/input/js0");

    // Need to find a way to allow people to be dumb and use
    // buttons for axes and viceversa
    // also to allow inverse for axes
    std::unordered_map<std::string, uint8_t> axes_map = {
        {"LLR", 0}, // Left thumbstick Left/right
        {"LUD", 1}, // Left thumbstick Up/Down
        {"LT", 2},  // Left trigger
        {"RLR", 3}, // Right thumbstick Left/Right
        {"RUD", 4}, // Right thumbstick Up/Down
        {"RT", 5},  // Right Trigger
        {"DP1", 6}, // D-Pad NOTE: PROBABLY DOESN'T WORK
        {"DP2", 7}, // D-Pad NOTE: PROBABLY DOESN'T WORK
    };

    std::unordered_map<std::string, uint8_t> buttons_map = {
        {"A", 0},
        {"B", 1},
        {"X", 2},
        {"Y", 3},
        {"LB", 4}, // Left Bumper
        {"RB", 5}, // Right Bumper
        {"SELECT", 6},
        {"START", 7},
        {"XB", 8}, // Xbox Button
        {"L3", 9}, // Click in left stick
        {"R3", 10}, // Click in right stick
    };

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
        if (_joyMsg.axes[2] < -0.5 && messageTime < 60)//trigger pressed
        {
            this->Arm();
            _manualArmed = true;
        }
    }
    else //armed
    {
    if (_joyMsg.axes[2] >= -0.5) // trigger not pressed
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
    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _joyMsg.axes[3]*-500 + MID_PWM);//right stick left-right
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _joyMsg.axes[4]*500 + MID_PWM);//right stick up-down
    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _joyMsg.axes[1]*500 + MID_PWM);//left stick up-down
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _joyMsg.axes[0]*-500 + MID_PWM);//left stick left-right

    //amateur hour
    //MavrosCommunicator->SetOverrideMessage()
}
