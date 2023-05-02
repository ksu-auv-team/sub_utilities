/**
https://github.com/ksu-auv-team/movement_package
sub_control_definitions.h
Purpose: Provides definitions for control and communication with mavros/mavlink

@author shadySource
@version 0.0.1
*/
#ifndef SUB_CONTROL_DEF
#define SUB_CONTROL_DEF

#define PITCH_CHAN      0
#define ROLL_CHAN 	    1
#define THROTTLE_CHAN 	2
#define YAW_CHAN 	    3
#define FORWARD_CHAN    4
#define LATERAL_CHAN    5

#define HIGH_PWM	1650
#define MID_PWM 	1500
#define LOW_PWM 	1350

#define ARM_OPEN  1650
#define ARM_CLOSE 1350
#define ARM_NEUTRAL 1500

#endif
