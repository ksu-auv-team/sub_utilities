#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
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

using namespace std;

ros::Publisher auv_pid_rc_override;

int x,y;
double dist,desiredDist;

void poseMessage(const std_msgs::Int32MultiArray& msg){
		x=msg.data[0];
        y=msg.data[1];
        dist=msg.data[2];
        desiredDist=msg.data[3];
}

int main( int argc, char** argv ){
    ros::init(argc, argv,"controller_calib_node");
	ros::NodeHandle nh;
    ros::Rate RC_COMM_RATE(45);
    ros::Subscriber sub_obj = nh.subscribe("controller_calib", 1000, &poseMessage);
    auv_pid_rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1000);
    mavros_msgs::OverrideRCIn MAV_MSG;
    PID calibrated_controller(HIGH_PWM,LOW_PWM,YAW_CHAN);  //PI(int inTopLimit,int inBottomLimit,int inChannel) 2-throttle 3-yaw 4-forward 5-lateral
    while (ros::ok())
    {
        //void setPID(bool inStatus,int inGoal, int inPose, int inMode);
        //inGoal should be the center of the cam so if 640x480 x_inGoal = 320 y_inGoal = 240
        //inPose is the location of the tracked object the x and y coordinates are treated in the callback function above
        //inMode is for the camera used 1- Forward Facing Camera 2- Downward Facin Camera
        calibrated_controller.setPID(true,320,x,1); 
        //Set the mavros commands here
        //We could use the PI::getCommand() function but we'll use the other control commands instead
        //It'll allow us to set the PI but also test the commands use in the actual code
        //PI::throttle_command()
        //PI::yaw_command()
        //PI::forward_command()
        //PI::lateral_command()
        MAV_MSG.channels[THROT_CHAN] = MID_PWM; 
		MAV_MSG.channels[YAW_CHAN] = calibrated_controller.yaw_command(); 							
		MAV_MSG.channels[FORWARD_CHAN] = MID_PWM; 
		MAV_MSG.channels[LATERAL_CHAN] = MID_PWM;
        auv_pid_rc_override.publish(MAV_MSG);		
        ros::spinOnce();
        RC_COMM_RATE.sleep();
    }
}