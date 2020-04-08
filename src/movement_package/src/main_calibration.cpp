#include "calibration_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_control_node");

    auto calibrationController = new controller::CalibrationController();

    calibrationController->Arm();

    calibrationController->ControlLoop();

}