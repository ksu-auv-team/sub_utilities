
#ifndef MANUAL_CONTROLLER_DEF
#define MANUAL_CONTROLLER_DEF

#include "controller.h"
#include <sensor_msgs/Joy.h>

#include <memory> // for shared ptr

namespace controller
{

class ManualController : public Controller
{
    private:
        bool _manualArmed;
        double _lastMsgRecieved;

        //@var _n nodehandle for ManualController
        ros::NodeHandle _n;

        //@var _joyStickSub subscriber for joystick message
        ros::Subscriber _joyStickSub;

        //@var _joyStickSub subscriber for joystick message
        sensor_msgs::Joy _joyMsg;

        /**
        Saves the joystick message into _joyMsg
        */
        void JoyStickCallback(const sensor_msgs::Joy& msg);

        /**
        Sets the Override Message from  _joyMsg.
        Polymorph from controller
        */
        void ProcessChannels();

        /**
        Manages armed status
        */
        void SafeArm();

        int _lateral = 3;
        int _forward = 4;
        int _throttle = 1;
        int _yaw = 0;
        int _arm = 2;
        int _armTimeoutSec = 60;

        bool _inverse_forward, _inverse_lateral, _inverse_yaw, _inverse_throttle = false;

    public:

    ManualController();
};

}

#endif