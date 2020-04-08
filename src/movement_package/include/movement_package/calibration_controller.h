
#ifndef AI_CONTROLLER_DEF
#define AI_CONTROLLER_DEF
#include "controller.h"
#include "pid.h"
#include <std_msgs/Float32MultiArray.h>

namespace controller
{
class CalibrationController : public Controller
{
    private:

        float _x, _y, _dist; 
        
        int _mode;

        ros::NodeHandle _nh;

        ros::Subscriber _calibrationSub;

        PID _calibrationController;

        void TargetCallback(const std_msgs::Float32MultiArray& msgs);

    public:

        void ProcessChannels();

    CalibrationController();

};

}

#endif