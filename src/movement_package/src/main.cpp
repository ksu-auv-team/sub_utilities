#include "mavros_communicator.h"
#include "controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "movement_package_node");

    auto sub_controller = new controller::Controller();

    sub_controller->Arm();

    sub_controller->ControlLoop();

    delete sub_controller;
}