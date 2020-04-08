#include "ai_controller.h"

using namespace controller;

AIController::AIController()
        : PERCENT_ERROR(5), _pressureCollected(false),
        _surfacePressure(101325)//1 atm
{
    _startTime = ros::Time::now().toSec();

    while(_startTime == 0)
    {
        ROS_INFO("TIME_ERROR");
        _startTime = ros::Time::now().toSec();
    }

    _targetSub = _nh.subscribe("/pi_loop_data", 10, &AIController::TargetCallback, this);

    _pressureSub = _nh.subscribe("/mavros/imu/atm_pressure", 1, &AIController::DepthCallback, this); //subscribe to arduino topic

    _setpointReachedPub = _nh.advertise<std_msgs::Bool>("pid_loop_check",10);

    _pastMode = -1;

    Arm();

    Sequencing();

    Disarm();

    MavrosCommunicator->SetModeStabilize();
}

AIController::~AIController()
{
    delete _throttleController;
    delete _yawController;
    delete _forwardController;
    delete _lateralController;
}

void AIController::TargetCallback(const std_msgs::Float32MultiArray& msg)
{
    _mode = round(msg.data[0]);
    for(int i(1); i < msg.data.size(); i++)
        _controlMsg[i-1] = msg.data[i];
    
}

void AIController::DepthCallback(const sensor_msgs::FluidPressure& msg)
{

    double currentTime = ros::Time::now().toSec();

	if ( (currentTime - _startTime) < 5.0d )
	{
		_pressureData.push_back(msg.fluid_pressure);
		
	}
    //average the data then set pressure_detected so it never averages again
    else if ( !_pressureCollected )
	{
        ROS_INFO("Finished collecting atmospheric pressure");
        //These are in Pa
	    //1 ATM = surface_pressure Pa
	    //collect data for first five seconds;
		
        int samplesize = _pressureData.size();
		
        if (samplesize == 0)
        {
			 ROS_WARN("NO Presssure Data Collected During First 3 Secs, defaulting to 1ATM");
        }
        else
		{
			_surfacePressure = 0;
			
            for (int i = 0; i < samplesize; i++)
            {
				_surfacePressure += _pressureData[i]/samplesize;
            }
        }
		_pressureCollected = true;
    } //calculated average
    else
    {
        //calculate current depth
        _currentDepth = (msg.fluid_pressure-_surfacePressure)*1.019744/10000;
        ROS_INFO("current_depth :  %f", _currentDepth);
        ROS_INFO("%d", Armed);
    }
}
void AIController::NewMode()
{
    delete _throttleController;
    delete _yawController;
    delete _forwardController;
    delete _lateralController;
    switch(_mode)
    {
        case TRACK_FRONT_HOLD_DEPTH:
            /*
                Throttle: (error in meters depth)
                p : 500/0.762:  full throt at 2.5 ft. err, 2.5ft~=.76 meters
                i : 100: 3 sec, 4 in error -> -100 pwm
                d : 600 : (1/2 ft delta error)/sec  -> -100 pwm
            */
            _throttleController = new PID(HIGH_PWM, LOW_PWM, MID_PWM, 500/0.762500/0.762, 0, 0);
            /*
                Yaw:
                p : 500: 1/2 throttle (250) in max image error (0.5)
                i : 25: should not be much continuous error over time...
                d : 300
            */
            _yawController = new PID(HIGH_PWM, LOW_PWM, MID_PWM, 500, 0, 0);

            break;
        case TRACK_FRONT:
            //todo
            break;
        case TRACK_BOTTOM_HOLD_DEPTH:
            //todo
            break;
        case FULL_SPEED_AHEAD:
            _throttleController = new PID(HIGH_PWM, LOW_PWM, MID_PWM, 500/.76, 0, 0);
            break;
    }
}

void AIController::ProcessChannels()
{
    
    if(_mode != _pastMode)
    {
        NewMode();
        _pastMode = _mode;
    }
    if (_pressureCollected)
    {
        switch(_mode)
        {
            case DISARM:
                if (Armed)
                {
                    this->Disarm();
                }
                break;
            case TRACK_FRONT_HOLD_DEPTH: //0 should be depth hold,with control channel2 acting as depth 
                if (!Armed)
                {
                    this->Arm();
                }
                _yawController->UpdatePID(true, 0, _controlMsg[1]);//depth hold, y is depth
                _throttleController->UpdatePID(true, _controlMsg[2], _currentDepth);
                MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN,_throttleController->GetCommand());
                MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _yawController->GetCommand());
                MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _controlMsg[3]*500 + 1500);
                MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _controlMsg[4]*500 + 1500);
                break;
            case TRACK_FRONT:	//1 should be forward facing camera 
            break;
            case TRACK_BOTTOM_HOLD_DEPTH:	//2 should be downard facing camera
                break;
            case FULL_SPEED_AHEAD: //5
                if (!Armed)
                {
                    this->Arm();
                }
                ROS_INFO("%f, %f",  _controlMsg[0], _currentDepth);
                _throttleController->UpdatePID(true, _controlMsg[0], _currentDepth);
                ROS_INFO("%f",_throttleController->GetCommand());
                 ROS_INFO("%f", 1500 - 500000/.76*(_controlMsg[0]-_currentDepth));
                MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _throttleController->GetCommand());
                MavrosCommunicator->SetOverrideMessage(YAW_CHAN, MID_PWM);
                MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, HIGH_PWM);
                MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, MID_PWM);
                break;
        }    
        // if((_throttleController->GetPercentError()<PERCENT_ERROR)&&
        //     (_yawController->GetPercentError()<PERCENT_ERROR)&&
        //     (_forwardController->GetPercentError()<PERCENT_ERROR)&&
        //     (_lateralController->GetPercentError()<PERCENT_ERROR))
        // {
        //     _setpointReached.data = true;
        // }
        // else
        // {
        //     _setpointReached.data = false;
        // }
        
        // _setpointReachedPub.publish(_setpointReached);
    }

}