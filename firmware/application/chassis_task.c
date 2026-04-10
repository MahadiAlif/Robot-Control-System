/**
  ****************************ROBOTO TEAM***************************************
  * @file       chassis_task.c/h
  * @brief      task employed to control the chassis of the robot
  *             
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ******************************************************************************
  */

#include "chassis_task.h"
#include "remote_control.h"
#include "referee_alg.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** chassis control time periods ***/
// to be set manually
uint32_t dt_std_chassis_ms 		= 1;		// control time period for chassis of standard robot
uint32_t dt_br_chassis_ms 		= 1;		// control time period for chassis of balancing robot
uint32_t dt_sentry_chassis_ms	= 1;		// control time period for chassis of sentry robot
// set automatically in the code
uint32_t dt_chassis_ms;								// chassis control time period (in milliseconds)
float dt_chassis;											// chassis control time period (in seconds)


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void chassis_task(void const *pvParameters){
	
	// ensure that INS_task got already executed at least one time
	while (ins_correct_angle[0] == 0 && ins_correct_angle[1] == 0 && ins_correct_angle[2] == 0);
	
	// set chassis control time period (in milliseconds)
#if IS_STD
	dt_chassis_ms = dt_std_chassis_ms;
#elif IS_BR1 || IS_BR2 || IS_LBR1 || IS_LBR2
	dt_chassis_ms = dt_br_chassis_ms;
#elif IS_SENTRY
	dt_chassis_ms = dt_sentry_chassis_ms;
#endif
	
	// set chassis control time period (in seconds)
	dt_chassis = SEC(dt_chassis_ms);
	
	// initialize referee of chassis power consumption limit
#if IS_POW_CONS_CHASSIS_ENABLED
	chassis_power_consumption_init();
#endif
	
	while(1)
	{
		
		switch (robot_control_mode){			// switch between control modes is done inside the gimbal task
			
			case REMOTE_CONTROLLER_MODE:
			case CHASSIS_FOLLOW_GIMBAL_PC_MODE:
			case FIXED_GIMBAL_PC_MODE:
				if (IS_BR1 || IS_BR2)
					br_chassis_control_loop();
				else if (IS_STD)
					std_chassis_control_loop();
				else if (IS_SENTRY)
					sentry_chassis_control_loop(robot_control_mode);
			break;
			
			case ERROR_STOP_MODE:
				CAN_cmd_chassis((int16_t) 0, (int16_t) 0, (int16_t) 0, (int16_t) 0);
			break;
		}
		
		osDelay(dt_chassis_ms);
	}
}