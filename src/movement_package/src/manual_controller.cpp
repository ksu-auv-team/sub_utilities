#include "manual_controller.h"
#include <unordered_map>
#include <string>

using namespace controller;

ManualController::ManualController()
{
    _joyStickSub = _n.subscribe("joy", 10, &ManualController::JoyStickCallback, this);

    _n.setParam("joy_node/dev", "/dev/input/js0");

    _joyMsg.axes = {-0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    _joyMsg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    bool override_controls = false;
    _n.getParam("/manual_controls/override_controls", override_controls);

    if (override_controls) {
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

        // There is def a better way of doing this but I'm drunk rn so enjoy this lambda
        auto parseControlSchema = [&, a_map = axes_map, b_map = buttons_map](std::string controller_input) {
            // first check if the control we want to override is in the axis map
            // it = iterator
            auto it = a_map.find(controller_input);
            // if the iterator has not reached the end of the map (meaning the key does exist)
            // maps have a "null pointer" that exists after the last index, .end() gives us that last index
            if (it != a_map.end()) {
                return std::make_shared<float>(_joyMsg.axes[a_map.at(controller_input)]);
            } else { // didn't find it in the axes map, try the buttons map
                auto it_2 = b_map.find(controller_input);
                if (it_2 != b_map.end()) {
                    return std::make_shared<float>(_joyMsg.buttons[b_map.at(controller_input)]);
                } else {
                    ROS_WARN_STREAM("Control input by name " << controller_input << " does not exist in the button layout!");
                }
            }
        };

        /**
         *  * /manual_controls/arm: LT
         * /manual_controls/frontback: RUD
         * /manual_controls/override_controls: True
         * /manual_controls/strafe: RLR
         * /manual_controls/vertical: LUD
         * /manual_controls/yaw: LLR
        */
        std::string forward, lateral, throttle, yaw;
        bool arm;
        _n.getParam("/manual_controls/frontback", forward);
        _n.getParam("/manual_controls/strafe", lateral);
        _n.getParam("/manual_controls/vertical", throttle);
        _n.getParam("/manual_controls/yaw", yaw);
        _n.getParam("/manual_controls/arm", arm);

        ROS_INFO_STREAM("forward " << forward <<  " lateral " << lateral << " throttle " << throttle << " yaw " << yaw);

        // UUUUUUUGGGGGGGGGGHHHHHHHHHHHH thought I could get around pointers but I guess not
        // Might just force axes to be axes and only worry about arming???
        // _forward = parseControlSchema(forward);
        // _lateral = parseControlSchema(lateral);
        // _throttle = parseControlSchema(throttle);
        // _yaw = parseControlSchema(yaw);
    }

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
    ROS_WARN_STREAM("_joyMsg " << _joyMsg.axes[2]);
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
        // Going to have to know if this is a button if we allow arm to be a button
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
    std::cout << "_forward " << *_forward << std::endl;
    std::cout << "_lateral " << *_lateral << std::endl;
    std::cout << "_throttle " << *_throttle << std::endl;
    std::cout << "_yaw " << *_yaw << std::endl;

    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _joyMsg.axes[3]*-500 + MID_PWM);//right stick left-right
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _joyMsg.axes[4]*500 + MID_PWM);//right stick up-down
    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _joyMsg.axes[1]*500 + MID_PWM);//left stick up-down
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _joyMsg.axes[0]*-500 + MID_PWM);//left stick left-right

    //amateur hour
    //MavrosCommunicator->SetOverrideMessage()
}