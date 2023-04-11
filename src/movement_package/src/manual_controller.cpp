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

        // THERE'S A BETTER WAY TO DO INVERSE FOR SPECIFIC DIRECTIONS BUT I AM LAZY
        std::unordered_map<std::string, uint8_t> axes_map = {
            {"LLR", 0},   // Left thumbstick Left/right
            {"iLLR", 0},  // INVERSE Left thumbstick Left/right
            {"LUD", 1},   // Left thumbstick Up/Down
            {"iLUD", 1},  // INVERSE left thumbstick Up/Down
            {"LT", 2},    // Left trigger
            {"RLR", 3},   // Right thumbstick Left/Right
            {"iRLR", 3},  // INVERSE Right thumbstick Left/Right
            {"RUD", 4},   // Right thumbstick Up/Down
            {"iRUD", 4},  // INVERSE Right thumbstick Up/Down
            {"RT", 5},    // Right Trigger
            {"DPLR", 6},  // D-Pad Left-Right
            {"iDPLR", 6}, // INVERSE D-Pad Left-Right
            {"DPUD", 7},  // D-Pad Up-Down
            {"iDPUD", 7}  // INVERSE D-Pad Up-Down
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
            {"R3", 10} // Click in right stick
        };

        // There is def a better way of doing this but I'm drunk rn so enjoy this lambda
        auto parseAxesControlSchema = [&, a_map = axes_map](std::string controller_input, uint8_t default_value) {
            // first check if the control we want to override is in the axis map
            auto it = a_map.find(controller_input);
            // if the iterator has not reached the end of the map (meaning the key does exist)
            // maps have a "null pointer" that exists after the last index, .end() gives us that last index
            if (it != a_map.end()) {
                return it->second;
            } else { // If we could not find the ID in the map,use the defaut value
                return default_value;
            }
        };

        // There is def a better way of doing this but I'm drunk rn so enjoy this lambda
        auto parsButtonControlSchema = [&, b_map = buttons_map](std::string controller_input, uint8_t default_value) {
            // first check if the control we want to override is in the axis map
            auto it = b_map.find(controller_input);
            // if the iterator has not reached the end of the map (meaning the key does exist)
            // maps have a "null pointer" that exists after the last index, .end() gives us that last index
            if (it != b_map.end()) {
                return it->second;
            } else { // If we could not find the ID in the map,use the defaut value
                return default_value;
            }
        };

        /**
         * /manual_controls/arm: LT
         * /manual_controls/frontback: RUD
         * /manual_controls/override_controls: True
         * /manual_controls/strafe: RLR
         * /manual_controls/vertical: LUD
         * /manual_controls/yaw: LLR
         * /manual_controls/open_gripper: X
         * /manual_controls/close_gripper: Y
        */
        std::string forward, lateral, throttle, yaw, arm, open_gripper, close_gripper;
        _n.getParam("/manual_controls/frontback", forward);
        _n.getParam("/manual_controls/strafe", lateral);
        _n.getParam("/manual_controls/vertical", throttle);
        _n.getParam("/manual_controls/yaw", yaw);
        _n.getParam("/manual_controls/arm", arm);
        _n.getParam("/manual_controls/open_gripper", open_gripper);
        _n.getParam("/manual_controls/close_gripper", close_gripper);

        ROS_INFO_STREAM("forward " << forward <<  " lateral " << lateral << " throttle " << throttle << " yaw " << yaw << " close_gripper " << close_gripper << " open_gripper " << open_gripper);
        _inverse_forward = tolower(forward[0]) == 'i' ? true : false;
        _forward  = parseAxesControlSchema(forward, _forward);

        _inverse_lateral = tolower(lateral[0]) == 'i' ? true : false;
        _lateral  = parseAxesControlSchema(lateral, _lateral);

        _inverse_throttle = tolower(throttle[0]) == 'i' ? true : false;
        _throttle = parseAxesControlSchema(throttle, _throttle);

        _inverse_yaw = tolower(yaw[0]) == 'i' ? true : false;
        _yaw      = parseAxesControlSchema(yaw, _yaw);
        _arm      = parseAxesControlSchema(arm, _arm);

        _open_gripper  = parseButtonControlSchema(open_gripper, _open_gripper);
        _close_gripper = parseButtonControlSchema(close_gripper, _close_gripper);
    }

    _n.getParam("/manual_controls/arm_timeout_sec", _armTimeoutSec);


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
        if (_joyMsg.axes[_arm] < -0.5 && messageTime < _armTimeoutSec)//trigger pressed
        {
            this->Arm();
            _manualArmed = true;
        }
    }
    else //armed
    {
        // Going to have to know if this is a button if we allow arm to be a button
        if (_joyMsg.axes[_arm] >= -0.5) // trigger not pressed
        {
            this->Disarm();
            _manualArmed = false;
        }
        else if(messageTime > _armTimeoutSec)
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
    if (_joyMsg.buttons[_close_gripper] || _joyMsg.buttons[_open_gripper]) {
        if (_joyMsg.buttons[_close_gripper]) {
            _srv.request.param2 = ARM_CLOSE;
        } else if (_joyMsg.buttons[_open_gripper]) {
            _srv.request.param2 = ARM_OPEN;
        }
    } else {
        _srv.request.param2 = ARM_NEUTRAL;
    }

    bool result = _arm_srv.call(_srv);

    int lateral  = (_joyMsg.axes[_lateral]  * (_inverse_lateral ? -1 : 1))  * -500;
    int forward  = (_joyMsg.axes[_forward]  * (_inverse_forward ? -1 : 1))  * 500;
    int throttle = (_joyMsg.axes[_throttle] * (_inverse_throttle ? -1 : 1)) * 500;
    int yaw      = (_joyMsg.axes[_yaw]      * (_inverse_yaw ? -1 : 1))      * -500;

    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, (lateral + MID_PWM)); // DEFAULT: right stick left-right
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, (forward + MID_PWM));  // DEFAULT right stick up-down
    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, (throttle + MID_PWM)); // DEFAULT left stick up-down
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, (yaw + MID_PWM)); //DEFAULT left stick left-right

    // MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _joyMsg.axes[3]*-500 + MID_PWM);//right stick left-right
    // MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _joyMsg.axes[4]*500 + MID_PWM);//right stick up-down
    // MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _joyMsg.axes[1]*500 + MID_PWM);//left stick up-down
    // MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _joyMsg.axes[0]*-500 + MID_PWM);//left stick left-right
}
