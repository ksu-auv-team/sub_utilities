
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

        std::shared_ptr<float> _forward, _lateral, _throttle, _yaw, _arm;

    public:

    ManualController();
};

}

#endif