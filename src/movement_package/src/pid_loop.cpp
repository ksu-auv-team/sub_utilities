#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/MultiArrayDimension.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include "pid.h"

#define THROT_CHAN 	2
#define YAW_CHAN 	3
#define FORWARD_CHAN 	4
#define LATERAL_CHAN	5
#define HIGH_PWM	1800
#define MID_PWM 	1500
#define LOW_PWM 	1200
#define PERCENT_ERR 5

ros::Publisher auv_pid_rc_override;
ros::Publisher pid_loop_check;

float x,y;
int mode;
double dist,desiredDist;

void poseMessage(const std_msgs::Float32MultiArray& msg){
		x=msg.data[0];
        y=msg.data[1];
        dist=msg.data[2];
        desiredDist=msg.data[3];
        mode=msg.data[4];
}

int main( int argc, char** argv ){
    ros::init(argc, argv,"Leviathan_PI_Controller_node");
	ros::NodeHandle nh;
    ros::Rate RC_COMM_RATE(45);
    ros::Subscriber sub_obj = nh.subscribe("pi_loop_data", 1000, &poseMessage);
    auv_pid_rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1000);
    pid_loop_check = nh.advertise<std_msgs::Bool>("pid_loop_check",1000);
    mavros_msgs::OverrideRCIn MAV_MSG;
    int past_mode = 0;
    std_msgs::Bool boolVar;
    PID throttle_controller(HIGH_PWM,LOW_PWM,THROT_CHAN);
	PID yaw_controller(HIGH_PWM,LOW_PWM,YAW_CHAN);
	PID forward_controller(HIGH_PWM,LOW_PWM,FORWARD_CHAN);
	PID lateral_controller(HIGH_PWM,LOW_PWM,LATERAL_CHAN);
	while (ros::ok())
    {
		if(mode != past_mode)
		{
			throttle_controller.reset();
			yaw_controller.reset();
			forward_controller.reset();
			lateral_controller.reset();
		}
        //Set PID based on the camera used
        switch(mode)
		{
			case 1:					//1 should be forward facing camera 
                throttle_controller.setPID(true,0,y,mode);
                yaw_controller.setPID(true,0,x,mode);
                forward_controller.setPID(true,desiredDist,dist,mode);
                lateral_controller.setPID(true,0,0,mode);
				break;
			case 2:					//2 should be downard facing camera
				throttle_controller.setPID(true,0,0,mode);
                yaw_controller.setPID(true,0,0,mode);
                forward_controller.setPID(true,0,y,mode);
                lateral_controller.setPID(true,0,x,mode);
                break;
		}
        MAV_MSG.channels[THROT_CHAN] = throttle_controller.throttle_command();
		MAV_MSG.channels[YAW_CHAN] = yaw_controller.yaw_command();							
		MAV_MSG.channels[FORWARD_CHAN] = forward_controller.forward_command();
		MAV_MSG.channels[LATERAL_CHAN] = lateral_controller.lateral_command();
		auv_pid_rc_override.publish(MAV_MSG);
        if((throttle_controller.getPercentError()<PERCENT_ERR)&&(yaw_controller.getPercentError()<PERCENT_ERR)&&(forward_controller.getPercentError()<PERCENT_ERR)&&(lateral_controller.getPercentError()<PERCENT_ERR))
            {
                boolVar.data=true;
                pid_loop_check.publish(boolVar);
            }
        else
            {
                boolVar.data=false;
                pid_loop_check.publish(boolVar);
            }
        ros::spinOnce();
        RC_COMM_RATE.sleep();

    }
    return 0;
}