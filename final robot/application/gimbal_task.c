/**
  ****************************ROBOTO TEAM***************************************
  * @file       gimbal_task.c/h
  * @brief      task employed to control the gimbal of the robot
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
 
#include "gimbal_task.h"
#include "remote_control.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** gimbal control time periods ***/
// to be set manually
uint32_t dt_std_gimbal_ms 		= 1;		// control time period for gimbal of standard robot
uint32_t dt_br_gimbal_ms 		= 1;			// control time period for gimbal of balancing robot
uint32_t dt_sentry_gimbal_ms	= 1;		// control time period for gimbal of sentry robot
// set automatically in the code
uint32_t dt_gimbal_ms;								// gimbal control time period (in milliseconds)
float dt_gimbal;											// gimbal control time period (in seconds)


/*** sentry state transitions ***/
static float time_sentry_state_transition = 0.0; //5e-2;					// time that should elapse between consecutive sentry state transitions
static float time_ms_last_sentry_state_transition = 0;


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/
  
void gimbal_task(void const *pvParameters){
	
	// ensure that INS_task got already executed at least one time
	while (ins_correct_angle[0] == 0 && ins_correct_angle[1] == 0 && ins_correct_angle[2] == 0);
	
	// set chassis control time period (in milliseconds)
#if IS_STD
	dt_gimbal_ms = dt_std_gimbal_ms;
#elif IS_BR2
	dt_gimbal_ms = dt_br_gimbal_ms;
#elif IS_SENTRY
	dt_gimbal_ms = dt_sentry_gimbal_ms;
#endif
	
	// set chassis control time period (in seconds)
	dt_gimbal = SEC(dt_gimbal_ms);
	
	
	while(TRUE)
	{

#if IS_SENTRY
		// perform sentry state transition (if needed)
		if (HAL_GetTick() - time_ms_last_sentry_state_transition >= time_sentry_state_transition*1e3) {
			
			sentry_state_transition();			// perform sentry state transition
			time_ms_last_sentry_state_transition = HAL_GetTick();			// save timestamp of last sentry state transition
		}
#endif
		
		if ( switch_is_down(rc_right_switch) ||
				(IS_SENTRY && sentry_state == IDLE)) {		
			
			// gimbal does NOT move
			CAN_cmd_gimbal((int16_t) 0, (int16_t) 0);
		}
		else {
			
			// gimbal control
			gimbal_control_loop();
		}
		
		osDelay(dt_gimbal_ms);
	}
}




