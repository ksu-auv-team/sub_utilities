#include "manual_controller.h"
#include <thread>
#include <chrono>
using namespace controller;

ManualController::ManualController()
{
    _joyStickSub = _n.subscribe("joy", 10, &ManualController::JoyStickCallback, this);

    _n.setParam("joy_node/dev", "/dev/input/js0");

    _joyMsg.axes = {-0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    _joyMsg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    this->Arm();

    this->Sequencing();

    this->Disarm();
    _manualArmed = false;
    _arm_srv = _n.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
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
        if (true/*_joyMsg.axes[2] < -0.5 && messageTime < 60*/)//trigger pressed
        {
            this->Arm();
            _manualArmed = true;
        }
    }
    else //armed
    {
    if (false)//_joyMsg.axes[2] >= -0.5) // trigger not pressed
     {
        this->Disarm();
        _manualArmed = false;
    }
    else if(false)// messageTime > 60)
    {
        this->Disarm();
        _manualArmed = false;
    }
    }
}

void ManualController::ProcessChannels()
{
    SafeArm();//trigger-arm
    if (_joyMsg.buttons[2]) {
    //     ROS_INFO_STREAM("_joyMsg.buttons[2] " << _joyMsg.buttons[2]);
    //     // send mavros command message
        // mavros_msgs::CommandLong srv;
        // for (;;) {
            // ROS_WARN_STREAM("sERVO " << i);
            _cmd_long.request.command = 183;
            _cmd_long.request.param1 = 8; // servos are 1-indexed here
            _cmd_long.request.param2 = 1450; // pwm
            _cmd_long.request.param3 = 0; // servos are 1-indexed here
            _cmd_long.request.param4 = 0; // servos are 1-indexed here
            _cmd_long.request.param5 = 0; // servos are 1-indexed here
            _cmd_long.request.param6 = 0; // servos are 1-indexed here
            _cmd_long.request.param7 = 0; // servos are 1-indexed here

            bool result = _arm_srv.call(_cmd_long);
            if (result) {
                ROS_INFO("DID IT WORK?");
            } else {
                ROS_INFO("it NO WORK");
            }
            // std::this_thread::sleep_for(std::chrono::seconds(3));
        // }
    }

    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _joyMsg.axes[3]*-500 + MID_PWM);//right stick left-right
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _joyMsg.axes[4]*500 + MID_PWM);//right stick up-down
    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _joyMsg.axes[1]*500 + MID_PWM);//left stick up-down
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _joyMsg.axes[0]*-500 + MID_PWM);//left stick left-right
    // MavrosCommunicator->SetOverrideMessage(ARM_CHAN, 1580);//left stick left-right

}
