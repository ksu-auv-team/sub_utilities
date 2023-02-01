#include <exception>
#include <stdexcept>
#include "manual_controller.h"

int main(int argc, char **argv)
{
    std::exception_ptr eptr;

    ros::init(argc, argv, "state_control_node");

    auto manualController = new controller::ManualController(2, 3, 1, 0, 5);

    try
    {
        manualController->ControlLoop();
    }
    catch (...)
    {
        manualController->Disarm();
    }

}