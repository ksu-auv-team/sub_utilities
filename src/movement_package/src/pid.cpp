#include <iostream>
#include "pid.h"

//#define pid 3.1415926 * d

PID::PID(float center)
{
	_center = center;
	_topLimit = std::numeric_limits<float>::max();
	_bottomLimit = std::numeric_limits<float>::min();
	_cl = clock();
	Reset();
}

PID::PID(float inTopLimit,float inBottomLimit, float center)
{	_center = center;
	_topLimit = inTopLimit;
	_bottomLimit = inBottomLimit;
	_cl = clock();
	Reset();
}

PID::PID(int inTopLimit,int inBottomLimit, float center, double kp, double ki, double kd)
{
	_center = center;
	_topLimit = inTopLimit;
	_bottomLimit = inBottomLimit;
	_cl = clock();
	SetGains(kp, ki, kd);
	Reset();
}


float PID::GetError()
{
	return _goal-_pose;
}

float PID::GetCommand()
{
	_dt = (clock() - _cl)/(double)CLOCKS_PER_SEC;
	_error = GetError();

	//	integrator anti windup here
	if (abs(_sigma) > _topLimit - _center) // uses top limit.
	{
		_sigma = 0;
	}

	_command = _center - (_kp*_error + _ki*_sigma + _kd*(_error-_lastError)/_dt);
	if(_command > _topLimit) //threshold the output
		_command = _topLimit;
	if(_command < _bottomLimit)
		_command = _bottomLimit;

	_lastError = _error;
	UpdateSigma();

	_cl = clock();
	return _command;
}

void PID::UpdatePID(bool inStatus,float inGoal, float inPose)
{
	_status = inStatus;				//inStatus has to be true
	SetGoal(inGoal);
	SetPose(inPose);
}

void PID::UpdateSigma()
{
	if(_status)
		_sigma += _error*_dt;
	else
	    _sigma = 0;
}

void PID::SetGains(double kp, double ki, double kd)
{
	_kp = kp; _ki = ki; _kd = kd;
}

void PID::SetGoal(float inGoal)
{
	_goal = inGoal;
}

void PID::SetPose(float inPose)
{
	_pose = inPose;
}

double PID::GetPercentError()
{
	if (_goal !=0)
		return 100*abs(GetError())/_goal;
	else
		return 100*abs(GetError())/eps;
}

void PID::Reset()
{
	_status = false;
	UpdateSigma();
}



