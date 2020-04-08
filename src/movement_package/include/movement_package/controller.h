/*
https://github.com/ksu-auv-team/movement_package
controller.h
Purpose: provides parent class for creating a controller. 
Controller.ProcessChannels should be overloaded for a new type ofcontroller.
I envision this being the parent of a manual controler object and
an AI/pid based controller object.

@author shadySource
@version 0.0.1
*/
#ifndef CONTROLLER_DEF
#define CONTROLLER_DEF

#include "mavros_communicator.h"

namespace controller
{

class Controller
{
    public:
        //@var MavrosCommunicator Object to interface with mavros
        mavcomm::MavrosCommunicator *MavrosCommunicator;

        bool Armed;

        //TODO : ground stateion connected subscriber, disarm if not subscribed
  
        /**
        Process channel inputs. Should be "hidden" for different controller types.
        Uses MavrosCommunicator->SetOverrideMessage to set rc channel outputs.
        Should also manage the mode of the FCU if necessary.

        @note default behavior is to set all inputs to MID_PWM.
        */
        virtual void ProcessChannels();


        /**
        Attempts to arm the FCU

        @return boolean success
        */
        bool Arm();

        /**
        Attempts to disarm the FCU

        @return boolean success
        */
        bool Disarm();

        /**
        calls ProcessChannels, Publishes the override message,
        and updates subscribers.
        */
        void ControlLoop();

        /**
        sequence to arm the esc
        */    
        void Sequencing();


    Controller();

    ~Controller();


};//end Controller


}//end namespace controller

#endif