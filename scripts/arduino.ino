#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

std_msgs::Bool is_killed;
ros::Publisher killswitch_pub("killswitch_status", &is_killed);

void setup() {
    ng.initNode();
    nh.advertise(killswitch_pub);
}

void loop() {
    if(digitalRead(4)){
        is_killed.data = false;
        killswitch_pub.publish(&is_killed);
    }
    else{
        is_killed.data = true;
        killswitch_pub.publish(&is_killed);
    }
}
