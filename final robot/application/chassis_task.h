#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "INS_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bmi088driver.h"
#include "CAN_receive.h"
#include "gimbal_control.h"
#include "br_chassis_control.h"
#include "control_util.h"
#include "std_chassis_control.h"
#include "shoot_rev_control.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** chassis control time periods ***/
extern uint32_t dt_chassis_ms;								// chassis control time period (in milliseconds)
extern float dt_chassis;											// chassis control time period (in seconds)


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

#endif
