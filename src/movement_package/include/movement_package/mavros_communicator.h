/**
https://github.com/ksu-auv-team/movement_package
mavros_communicator.h
Purpose: Provides a class with members for communicating to mavros

@author shadySource
@version 0.0.1
*/
#ifndef MAVROS_COMMNUICATOR_DEF
#define MAVROS_COMMNUICATOR_DEF

#include <chrono>
#include <string.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/StreamRate.h>
#include <sub_control_definitions.h>

namespace mavcomm
{

class MavrosCommunicator
{
    private:
        //@var _n NodeHandle for ros communication to mavros
        ros::NodeHandle _n;

        //@var INFO_RATE stream rate from FCU
        const int INFO_RATE;

        // Service Params for setting FCU modes, states, params, etc.
        ros::ServiceClient _streamRateSrv, _modeSrv, _armSrv, _paramSrv;
        mavros_msgs::SetMode _stabilizeModeMsg, _altHoldModeMsg, _acroModeMsg, _manualModeMsg;
        mavros_msgs::CommandBool _armMsg, _disarmMsg;
        mavros_msgs::ParamSet _sysidMsg;
        mavros_msgs::StreamRate _streamRateMsg;        

        //@var RC Override Message. modify channels array.
        //ex. OverrideMessage.channels[0] = ...
        // There are 8 channels.
        mavros_msgs::OverrideRCIn _overrideMessage;

        //@var Publisher for RC Override
        ros::Publisher _overridePub;

    public:
        //@var _fcuCommRate rate to send commands to the FCU
        ros::Rate FCUCommRate;


        /**
        Sets the override message. Message layout for ardusub v3.5
        see https://www.ardusub.com/operators-manual/rc-input-and-output.html
        
        @note: Bounds the provided values between LOW_PWM and HIGH_PWM

        @param roll - pwm value to assign to roll channel
        @param pitch - pwm value to assign to pitch channel
        @param yaw - pwm value to assign to yaw channel
        @param throttle - pwm value to assign to throttle channel
        @param forward - pwm value to assign to forward channel
        @param lateral - pwm value to assign to lateral channel
        */
        void SetOverrideMessage(const int &roll, const int &pitch, const int &yaw, const int &throttle, const int &forward, const int &lateral);


        /**
        Sets overridemessage at <idx> to <pwm>

        @note: Bounds the provided pwm value between LOW_PWM and HIGH_PWM
    
        @param idx - index in the message
        @param pwm - pwm to set the channel to
        */
        void SetOverrideMessage(const int &idx, const int &pwm);


        /**
        Sets the override mesage to all MID_PWM
        */
        void SetOverrideMessage();


        /**
        Publishes the override message.
        */
        void PublishOverrideMessage();

        /**
        Sets SYSID_MYGCS to 1 and FCU stream rate to 10.
        SYSID_MYGCS must be set to 1 for OverriceRCIn to function.

        @return true if set SYSID_MYGCS succeeds,
        false if set SYSID_MYGCS fails
        */
        bool CommInit();


        /**
        Arms the FCU

        @return boolean success
        */
        bool ArmFCU();


        /**
        Disarms the FCU

        @return boolean success
        */
        bool DisarmFCU();


        /**
        Sets FCU mode to "ACRO"

        @return boolean success
        */
        bool SetModeAcro();


        /**
        Sets FCU Mode to "STABILIZE"

        @return boolean success
        */
        bool SetModeStabilize();

        /**
        Sets FCU Mode to "ALT_HOLD", altitude (depth) hold mode

        @return boolean success
        */
        bool SetModeAltHold();

        /**
        Sets FCU Mode to "MANUAL"

        @return boolean success
        */
        bool SetModeManual();

        /**
        Checks motors by activating each control channel for 1 second.
        ex. pitch forward 1 sec, roll right 1 sec, ...

        @return bool success
        */
        bool MotorTest();

 
    MavrosCommunicator();

    ~MavrosCommunicator();

};

}

#endif