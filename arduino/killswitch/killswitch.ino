#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

std_msgs::Bool is_killed;
std_msgs::Bool run_start;
ros::Publisher killswitch_status_pub("killswitch_is_killed", &is_killed);
ros::Publisher killswitch_start_pub("killswitch_run_start", &run_start);

bool last_val = false;
bool curr_val = false;

void setup() {
    // Starut Nodes
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(killswitch_status_pub);
    nh.advertise(killswitch_start_pub);
    
    // Initialize Values
    pinMode(4, INPUT);
    curr_val = digitalRead(4);
    last_val = curr_val;
}

void loop() {
    // Read in Data from pin 4
    curr_val = digitalRead(4);
    
    // Publish real-time data
    if(curr_val){
        is_killed.data = false;
        killswitch_status_pub.publish(&is_killed);
        if(!last_val){ // If we went from off to on...
            run_start.data = true;
            killswitch_start_pub.publish(&run_start);
        }
    }
    else{
        is_killed.data = true;
        killswitch_status_pub.publish(&is_killed);
        if(last_val){ // If we went from on to off...
            run_start.data = false;
            killswitch_start_pub.publish(&run_start);
        }
    }

    // Reset last_val to curr_val
    last_val = curr_val;

    nh.spinOnce();
    delay(50);
}
