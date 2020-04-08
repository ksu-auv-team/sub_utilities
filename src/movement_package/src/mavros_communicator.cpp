#include "mavros_communicator.h"

using namespace mavcomm;

MavrosCommunicator::MavrosCommunicator()
    : INFO_RATE(10), FCUCommRate(45)
{
    // Publishers
    _overridePub = _n.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
    
    // Level out the OverrideRCIn message
    this->SetOverrideMessage();

    // Services
    _modeSrv = _n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _armSrv = _n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _paramSrv = _n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    _streamRateSrv = _n.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    
    _sysidMsg.request.param_id = "SYSID_MYGCS";
    _sysidMsg.request.value.integer = 1;
    _streamRateMsg.request.stream_id = 0;
    _streamRateMsg.request.message_rate = INFO_RATE;
    _streamRateMsg.request.on_off = true;
    _manualModeMsg.request.base_mode = 0;
    _manualModeMsg.request.custom_mode = "MANUAL";
    _stabilizeModeMsg.request.base_mode = 0;
    _stabilizeModeMsg.request.custom_mode = "STABILIZE";
    _altHoldModeMsg.request.base_mode = 0;
    _altHoldModeMsg.request.custom_mode = "ALT_HOLD";
    _acroModeMsg.request.base_mode = 0;
    _acroModeMsg.request.custom_mode = "ACRO";
    _armMsg.request.value = true;
    _disarmMsg.request.value = false;
}

MavrosCommunicator::~MavrosCommunicator()
{
    DisarmFCU();
}

void MavrosCommunicator::SetOverrideMessage(const int &roll, const int &pitch, const int &yaw, const int &throttle, const int &forward, const int &lateral)
{
    _overrideMessage.channels[ROLL_CHAN] = std::min(std::max(roll, LOW_PWM), HIGH_PWM);
    _overrideMessage.channels[PITCH_CHAN] = std::min(std::max(pitch, LOW_PWM), HIGH_PWM);
    _overrideMessage.channels[YAW_CHAN] = std::min(std::max(yaw, LOW_PWM), HIGH_PWM);
    _overrideMessage.channels[THROTTLE_CHAN] = std::min(std::max(throttle, LOW_PWM), HIGH_PWM);
    _overrideMessage.channels[FORWARD_CHAN] = std::min(std::max(forward, LOW_PWM), HIGH_PWM);
    _overrideMessage.channels[LATERAL_CHAN] = std::min(std::max(lateral, LOW_PWM), HIGH_PWM);
}

void MavrosCommunicator::SetOverrideMessage(const int &idx, const int &pwm)
{
    if (idx >= 0 && idx <= 7)
    {
        _overrideMessage.channels[idx] = std::min(std::max(pwm, LOW_PWM), HIGH_PWM);
    }
    else
    {
        ROS_WARN("SetOverrideMessage Failed: idx out of bounds.");
    }
}

void MavrosCommunicator::SetOverrideMessage()
{
    _overrideMessage.channels[0] = MID_PWM;
    _overrideMessage.channels[1] = MID_PWM;
    _overrideMessage.channels[2] = MID_PWM;
    _overrideMessage.channels[3] = MID_PWM;
    _overrideMessage.channels[4] = MID_PWM;
    _overrideMessage.channels[5] = MID_PWM;
    _overrideMessage.channels[6] = MID_PWM;
    _overrideMessage.channels[7] = MID_PWM;
}

void MavrosCommunicator::PublishOverrideMessage()
{
    _overridePub.publish(_overrideMessage);
}


bool MavrosCommunicator::CommInit()
{
    if (_streamRateSrv.call(_streamRateMsg))
    {
        ROS_INFO("Stream rate set to %d.", _streamRateMsg.request.message_rate);
    }
    else
    {
        ROS_WARN("Failed to use mavros/set_stream_rate service. Rate not set.");
        return false;
    }

    if (_paramSrv.call(_sysidMsg))
    {
        if (_sysidMsg.response.success)
        {
            ROS_INFO("Set SYSID_MYGCS Succeeded.");
        }
        else
        {
            ROS_WARN("Set SYSID_MYGCS Failed.");
        }
        return _sysidMsg.response.success;
    }

    ROS_WARN("Failed to use mavros/param/set service. SYSID_MYGCS not set. RC Override will FAIL.");
    return false;
}

bool MavrosCommunicator::ArmFCU()
{
    if (_armSrv.call(_armMsg))
    {
        if (_armMsg.response.success)
        {
            ROS_INFO("Arm succeeded.");
        }
        else
        {
            ROS_WARN("Arm failed.");
        }
        return _armMsg.response.success;
    }
    ROS_WARN("Failed to use /mavros/cmd/arming serivce. Arm failed.");
    return false;
}

bool MavrosCommunicator::DisarmFCU()
{
    if (_armSrv.call(_disarmMsg))
    {
        if (_disarmMsg.response.success)
        {
            ROS_INFO("Disarm succeeded.");
        }
        else
        {
            ROS_WARN("Disarm failed.");
        }
        return _disarmMsg.response.success;
    }
    ROS_WARN("Failed to use /mavros/cmd/arming serivce. Disarm failed.");
    return false;
}

bool MavrosCommunicator::SetModeAcro()
{
    if (_modeSrv.call(_acroModeMsg))
    {
        if (_acroModeMsg.response.mode_sent)
        {
            ROS_INFO("Set mode to ACRO succeeded.");
        }
        else
        {
            ROS_WARN("Set mode to ACRO failed.");
        }
        return _acroModeMsg.response.mode_sent;
    }
    ROS_WARN("Failed to use /mavros/set_mode serivce. Set mode to ACRO failed.");
    return false;
}

bool MavrosCommunicator::SetModeStabilize()
{
    if (_modeSrv.call(_stabilizeModeMsg))
    {
        if (_stabilizeModeMsg.response.mode_sent)
        {
            ROS_INFO("Set mode to STABILIZE succeeded.");
        }
        else
        {
            ROS_WARN("Set mode to STABILIZE failed.");
        }
        return _stabilizeModeMsg.response.mode_sent;
    }
    ROS_WARN("Failed to use /mavros/set_mode serivce. Set mode to STABILIZE failed.");
    return false;
}

bool MavrosCommunicator::SetModeAltHold()
{
    if (_modeSrv.call(_altHoldModeMsg))
    {
        if (_altHoldModeMsg.response.mode_sent)
        {
            ROS_INFO("Set mode to ALT_HOLD succeeded.");
        }
        else
        {
            ROS_WARN("Set mode to ALT_HOLD failed.");
        }
        return _altHoldModeMsg.response.mode_sent;
    }
    ROS_WARN("Failed to use /mavros/set_mode serivce. Set mode to STABILIZE failed.");
    return false;
}

bool MavrosCommunicator::SetModeManual()
{
    if (_modeSrv.call(_manualModeMsg))
    {
        if (_manualModeMsg.response.mode_sent)
        {
            ROS_INFO("Set mode to MANUAL succeeded.");
        }
        else
        {
            ROS_WARN("Set mode to MANUAL failed.");
        }
        return _manualModeMsg.response.mode_sent;
    }
    ROS_WARN("Failed to use /mavros/set_mode serivce. Set mode to MANUAL failed.");
    return false;
}

bool MavrosCommunicator::MotorTest()
{
    bool _success = this->SetModeManual();
    if (!_success)
    {
        ROS_ERROR("Exiting MotorTest. See previous warning.");
        return false;
    }
    int i, activeChannel;
    auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end-start;
    while(ros::ok())
    {
        end = std::chrono::system_clock::now();
        diff = end-start;
        activeChannel = (int) diff.count();
        ROS_INFO("%d | %f", activeChannel, diff.count());
        if (activeChannel == 6) // break if not a valid rc direction command.
        {
            break;
        }
        this->SetOverrideMessage();//clear channels
        this->SetOverrideMessage(activeChannel, HIGH_PWM);
        this->PublishOverrideMessage();
        FCUCommRate.sleep();
    }
    this->DisarmFCU();
    return true;
}