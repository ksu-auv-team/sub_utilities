#include "ros/ros.h"
#include <ros/console>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>

#define ROLL_CHAN 	0
#define PITCH_CHAN 	1
#define THROT_CHAN 	2
#define YAW_CHAN 	3
#define MODES_CHAN	4

#define HIGH_PWM	1900
#define MID_PWM 	1500
#define LOW_PWM 	1100

using namespace std;

ros::Publisher auv_pid_rc_override;

int target [2]; 
int mode;

void yolo_callback(const [data_type::data specific::ConstPtr& msg])
{
    for (int i=0; i < 3; i++) //recieving data type f 1x2
    {	
		//Give the first 2 data values of the msg to target
        if i < 2
			target[i] = msg->[label of data];
		//Give the last value to mode
		//1 should be forward facing camera
		//2 should be downard facing camera
        if i == 2
			mode = msg->[label of data];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auv_pid");
    ros::NodeHandle nh;
    ros::Rate RC_COMM_RATE(45);
    ros::Subscriber yolo_input = nh.subscribe("name of yolo", 1000, &yolo_callback); //
    auv_pid_rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1000);
    int past_mode = 0;
    mavros_msgs::OverrideRCIn MAV_MSG;
    PI roll_controller(1900,1100,1);
	PI pitch_controller(1900,1100,2);
	PI throttle_controller(1900,1100,3);
	PI yaw_controller(1900,1100,4);
	while (ros::ok())
    {
		if(mode != past_mode)
		{
			roll_controller.reset();
			pitch_controller.reset();
			throttle_controller.reset();
			yaw_controller.reset();
		}
		switch(mode)
		{
			case 1:					//1 should be forward facing camera 
				MAV_MSG.channels[ROLL_CHAN] = roll_controller.roll_command();
				MAV_MSG.channels[PITCH_CHAN] = pitch_controller.pitch_command();							
				MAV_MSG.channels[THROT_CHAN] = throttle_controller.throttle_command();
				MAV_MSG.channels[YAW_CHAN] = yaw_controller.yaw_command();
				break;
			case 2:					//2 should be downard facing camera
				MAV_MSG.channels[ROLL_CHAN] = roll_controller.roll_command();
				MAV_MSG.channels[PITCH_CHAN] = pitch_controller.pitch_command();							
				MAV_MSG.channels[THROT_CHAN] = throttle_controller.throttle_command();
				MAV_MSG.channels[YAW_CHAN] = yaw_controller.yaw_command();
				break;
		}
		MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
		auv_pid_rc_override.publish(MAV_MSG);
		ros::spinOnce();
        RC_COMM_RATE.sleep;      
    }
    return 0;
    }
