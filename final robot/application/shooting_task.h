#ifndef SHOOTING_TASK_H
#define SHOOTING_TASK_H

#include "struct_typedef.h"
#include "INS_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bmi088driver.h"
#include "CAN_receive.h"
#include "gimbal_control.h"
#include "br_chassis_control.h"
#include "remote_control.h"
#include "shoot_rev_control.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

extern uint8_t shooting_control_mode;						// current shooting control mode
extern uint8_t shooting_control_mode_prev;			// previous shooting control mode


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

#endif