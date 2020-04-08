#include "ros/ros.h"
#include <ros/console>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include "pid.h"

#define ROLL_CHAN 	0
#define PITCH_CHAN 	1
#define THROT_CHAN 	2
#define YAW_CHAN 	3
#define MODES_CHAN	4

#define HIGH_PWM	2000
#define MID_PWM 	1500
#define LOW_PWM 	1000

ros::Publisher auv_pid_rc_override;

void yolo_callback(const [data_type::data specific::ConstPtr& msg])
{	
	//Define target's coordinates
	//x-coordinate = target[1] and y-coordinate = target[2]
	int target [2]; 
    for (int i=0; i < 2; i++) //recieving data type f 1x2
    {
        target[i] = msg->[label of data];
    }
    mavros_msgs::OverrideRCIn MAV_MSG;
	//Define our origin based on size of picture
	//For example a (640x480)camera will give us int origin [2] = {320,240}; 
	int origin [2];
	//Define PI gains and variables
	float Kp_x =1;float Kp_y =1;   //Proportional gains for controller
	float Ki_x =0;float Ki_y =1;   //Integral gains for controller
	int err_x =0;err_y=0;		   //Error in x and y
	long acc_x = 0;long acc_y = 0; //Accumulator
	int PWM_x,PWM_y;               //Calculated PWM values
	err_x = origin[1]-target[1];err_y = origin[2]-target[2];        //Errors are calculated
	//PWM Values are calculated
	PWM_x = 1500-(Kp_x*err_x+Ki_x*acc_x);							
	PMW_y = 1500-(Kp_y*err_y+Ki_y*acc_y);
	//The values should not exceed our limits of 2000 and 1000 so if-statements are used to avoid that
	if(PWM_x>2000)
		PWM_x = 2000;
	if(PWM_y>2000)
		PWM_y = 2000;
	if(PWM_x<1000)
		PWM_x = 1000;
	if(PWM_y<1000)
		PWM_y = 1000;	
	acc_x += err_x;
	acc_y += err_y; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auv_pid");
    ros::NodeHandle nh;
    ros::Rate RC_COMM_RATE(45);
    ros::Subscriber yolo_input = nh.subscriber("name of yolo", 1000, &yolo_callback); //
    auv_pid_rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1000);
	while (ros::ok())
    {
		ros::spinOnce();
		MAV_MSG.channels[ROLL_CHAN] = ;
		MAV_MSG.channels[PITCH_CHAN] = ;							//Set to 1500 to PID without the sub moving forward
		MAV_MSG.channels[THROT_CHAN] = ;
		MAV_MSG.channels[YAW_CHAN] = ;
		MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
		auv_pid_rc_override.publish(MAV_MSG);
        RC_COMM_RATE.sleep;      
    }
    return 0;
}
