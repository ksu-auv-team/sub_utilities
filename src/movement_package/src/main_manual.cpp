#include <exception>
#include <stdexcept>
#include "manual_controller.h"

int main(int argc, char **argv)
{
    std::exception_ptr eptr;

    ros::init(argc, argv, "manual_control_node");

    auto manualController = new controller::ManualController();

    try
    {
        manualController->ControlLoop();
    }
    catch (...)
    {
        manualController->Disarm();
    }

}