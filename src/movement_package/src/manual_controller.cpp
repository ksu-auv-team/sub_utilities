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

    /*
        So according to this site: https://ardupilot.org/copter/docs/common-servo.html, the servos are indexed starting at 0 not 1.
        Its located in the following section: Controlling the servo as a servo.

        So in our code, writing to Pin 9 is actually the SERVO8_FUNCTION in QGroundControl. This might cause some problems when
        going back to Meta Knight.
    */

    /*
        Below sets up the command that needs to be sent to the pixhawk based on user input.
    */
    _srv.request.command = 183; // command to send. should link to MAV_CMD_DO_SET_SERVO
    _srv.request.param1 = 9; // pins on the pixhawk are 1 - indexed according to documentation
    _srv.request.param3 = 0;
    _srv.request.param4 = 0;
    _srv.request.param5 = 0;
    _srv.request.param6 = 0;
    _srv.request.param7 = 0;

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

    /*
        If either Xbox button X or Y is pressed change the second parameter to match the needed PWM value
    */
    if (_joyMsg.buttons[2] || _joyMsg.buttons[3]) {
        if (_joyMsg.buttons[2]) {
            _srv.request.param2 = ARM_CLOSE;
        } else if (_joyMsg.buttons[3]) {
            _srv.request.param2 = ARM_OPEN;
        }
    } else {
        srv.request.param2 = ARM_NEUTRAL;
    }

    bool result = _arm_srv.call(_srv);

    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _joyMsg.axes[3]*-500 + MID_PWM);//right stick left-right
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _joyMsg.axes[4]*500 + MID_PWM);//right stick up-down
    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _joyMsg.axes[1]*500 + MID_PWM);//left stick up-down
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _joyMsg.axes[0]*-500 + MID_PWM);//left stick left-right
}
