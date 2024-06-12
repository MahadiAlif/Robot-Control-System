/* task for shooting */

#include "shooting_task.h"
#include "referee_alg.h"
#include "remote_control.h"


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
uint32_t dt_std_shoot_rev_ms 		= 2;		// control time period for shooting and rev motors of standard robot
uint32_t dt_br_shoot_rev_ms 		= 2;		// control time period for shooting and rev motors of balancing robot
uint32_t dt_sentry_shoot_rev_ms	= 2;		// control time period for shooting and rev motors of sentry robot
// set automatically in the code
uint32_t dt_shoot_rev_ms;								// shooting and rev motors control time period (in milliseconds)
float dt_shoot_rev;											// shooting and rev motors control time period (in seconds)
uint32_t pressed_mouse_key;
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
	
	while(TRUE)
	{
		
		/* set previous shooting mode */
		shooting_control_mode_prev = shooting_control_mode;
		
		if(switch_is_up(rc_right_switch))		// REMOTE CONTROLLER SHOOTING MODE
		{
			
			/* set current shooting mode */
			if (rc_up_left_wheel >= 600) {
				
				shooting_control_mode = MULTIPLE_SHOOT_MODE;
			}
			else if (rc_up_left_wheel <= -100) {
				
				shooting_control_mode = SINGLE_SHOOT_MODE;
			}
			else if (rc_up_left_wheel >= 100 && rc_up_left_wheel <= 599) {
					
				shooting_control_mode = TRIPLE_SHOOT_MODE;
			}
			else {
				
				shooting_control_mode = NO_SHOOT_MODE;
			}
			
		}
		else if(switch_is_mid(rc_right_switch))		// PC SHOOTING MODE
		{
			if (IS_SENTRY) {
				
				shooting_control_mode = MULTIPLE_SHOOT_MODE;
			}
			else {
				
				if(is_mouse_raising_edge(MOUSE_LEFT_KEY))
					pressed_mouse_key = HAL_GetTick();
					
				else if(is_mouse_falling_edge(MOUSE_LEFT_KEY) && ((HAL_GetTick() - pressed_mouse_key) < 0.2e3)) {
					
					if (is_mouse_right_key_pressed())
						shooting_control_mode = SINGLE_SHOOT_MODE;
					
					else
						shooting_control_mode = TRIPLE_SHOOT_MODE;
				}
				
				else if (is_mouse_left_key_pressed() && ((HAL_GetTick() - pressed_mouse_key) > 0.2e3))
					shooting_control_mode = MULTIPLE_SHOOT_MODE;
				
				else 
					shooting_control_mode = NO_SHOOT_MODE;
			}
		}
		
		switch (shooting_control_mode)
		{
//			case NO_SHOOT_MODE:
//				CAN_cmd_shoot((int16_t) 0, (int16_t) 0, (int16_t) 0);
//			break;
			
			case NO_SHOOT_MODE:
			case SINGLE_SHOOT_MODE:
			case TRIPLE_SHOOT_MODE:
			case MULTIPLE_SHOOT_MODE:
				shoot_rev_control_loop(shooting_control_mode, shooting_control_mode_prev);
			break;
		}
		
		osDelay(2);
	}
}

