#include "ai_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ai_control_node");

    auto aiController = new controller::AIController();

    //aiController->Arm();

    aiController->ControlLoop();

}