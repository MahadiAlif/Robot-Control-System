/* task for shooting */

#include "shooting_task.h"
#include "referee_alg.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** shooting control mode (determines the way you shoot, e.g., single shoot, multiple shoot ...) ***/
uint8_t shooting_control_mode = NO_SHOOT_MODE;		// current shooting control mode (initialized to NO_SHOOT_MODE for safety)
uint8_t shooting_control_mode_prev;								// previous shooting control mode

/*** shoot and rev control time periods ***/
// to be set manually
uint32_t dt_std_shoot_rev_ms 		= 5;		// control time period for shooting and rev motors of standard robot
uint32_t dt_br_shoot_rev_ms 		= 5;		// control time period for shooting and rev motors of balancing robot
uint32_t dt_sentry_shoot_rev_ms	= 5;		// control time period for shooting and rev motors of sentry robot
// set automatically in the code
uint32_t dt_shoot_rev_ms;								// shooting and rev motors control time period (in milliseconds)
float dt_shoot_rev;											// shooting and rev motors control time period (in seconds)

// TODO add the above to .h and in the osDelay at the bottom of this file

/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void shooting_task(void const *pvParameters){
	
	while (abs(rc_up_left_wheel) > 5)			// we check that at the start the shooting motor are off to avoid injuries
	{
		osDelay(2);
	}
	
	// initialize referee for shooting speed
#if IS_POW_CONS_SHOOT_FREQ_ENABLED || IS_POW_CONS_SHOOT_SPEED_ENABLED
	shoot_speed_init();
#endif
	
	while(1)
	{
		/* set previous shooting mode */
		shooting_control_mode_prev = shooting_control_mode;
		
		/* set current shooting mode */
		if (rc_up_left_wheel >= 450)
			shooting_control_mode = SINGLE_SHOOT_MODE;
		else if (rc_up_left_wheel <= -450)
			shooting_control_mode = MULTIPLE_SHOOT_MODE;
		else
			shooting_control_mode = NO_SHOOT_MODE;
			
		//shooting_control_mode = get_remote_control_point()->rc.s[1];
		
		switch (shooting_control_mode)
		{
//			case NO_SHOOT_MODE:
//				CAN_cmd_shoot((int16_t) 0, (int16_t) 0, (int16_t) 0);
//			break;
			
			case NO_SHOOT_MODE:
			case SINGLE_SHOOT_MODE:
			case MULTIPLE_SHOOT_MODE:
				shoot_rev_control_loop(shooting_control_mode, shooting_control_mode_prev);
			break;
		}
		
		osDelay(5);
	}
}

