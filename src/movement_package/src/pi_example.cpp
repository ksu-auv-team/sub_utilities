#include <iostream>
#include "pid.h"

using namespace std;

int main()
{
	PID yaw_controller(1900,1100,4);
	while(1)
	{
		yaw_controller.setPID(true,320,640,1);
		//cout <<yaw_controller.kp <<" " <<yaw_controller.ki <<" " <<yaw_controller.mode <<" " <<yaw_controller.channel <<endl; 
		cout <<yaw_controller.getError() <<" "  <<yaw_controller.getCommand() <<" " <<yaw_controller.yaw_command() <<endl;
	}
}
