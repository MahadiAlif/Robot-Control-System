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
uint32_t dt_br_chassis_ms 		= 4;		// control time period for chassis of balancing robot
uint32_t dt_sentry_chassis_ms	= 1;		// control time period for chassis of sentry robot
// set automatically in the code
uint32_t dt_chassis_ms;								// chassis control time period (in milliseconds)
float dt_chassis;											// chassis control time period (in seconds)

static uint32_t chassis_task_start_timestamp_ms;
static uint32_t chassis_task_end_timestamp_ms;
static uint32_t chassis_task_elapsed_time_ms;
static uint32_t chassis_task_sleep_time_ms;

/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void chassis_task(void const *pvParameters){
	
	// ensure that INS_task got already executed at least one time
	while (ins_correct_angle[0] == 0 && ins_correct_angle[1] == 0 && ins_correct_angle[2] == 0);
	
	// set chassis control time period (in milliseconds)
#if IS_STD
	dt_chassis_ms = dt_std_chassis_ms;
#elif IS_BR2
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
	
	while(TRUE)
	{
		chassis_task_start_timestamp_ms = HAL_GetTick();
		
		if ( switch_is_down(rc_right_switch) ||
				(IS_SENTRY && sentry_state == IDLE)) {		
			
			// chassis does NOT move
			CAN_cmd_chassis((int16_t) 0, (int16_t) 0, (int16_t) 0, (int16_t) 0);
		}
		else {
			
#if IS_STD || IS_SENTRY
			std_chassis_control_loop();
#elif IS_BR2
			br_chassis_control_loop();
#endif
		}
		
		chassis_task_end_timestamp_ms = HAL_GetTick();
		chassis_task_elapsed_time_ms = chassis_task_end_timestamp_ms - chassis_task_start_timestamp_ms;
		chassis_task_sleep_time_ms = max(dt_chassis_ms - chassis_task_elapsed_time_ms, (uint32_t) 0);
		
		osDelay(dt_chassis_ms);
	}
}