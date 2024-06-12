#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "INS_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bmi088driver.h"
#include "CAN_receive.h"
#include "gimbal_control.h"
#include "br_chassis_control.h"
#include "remote_control.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** gimbal control time periods ***/
extern uint32_t dt_gimbal_ms;								// gimbal control time period (in milliseconds)
extern float dt_gimbal;											// gimbal control time period (in seconds)


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

#endif
