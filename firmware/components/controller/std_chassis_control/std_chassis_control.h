#ifndef STD_CHASSIS_CONTROL_H
#define STD_CHASSIS_CONTROL_H

#include <stdint.h>

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

#define NUM_STD_WHEELS 4			// number of wheels of standard robot chassis


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

/************************************************************************************************************************
	NAME: std_chassis_control_loop
	
	DESCRIPTION:
	- implements the feedback-control loop for the chassis of standard robot
	- main operations:
		1. acquires the state feedback from sensors
		2. computes the control signals
		3. sends the control signals to the motors of the robot
************************************************************************************************************************/
void std_chassis_control_loop(void);

#endif



