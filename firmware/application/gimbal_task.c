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

  
void gimbal_task(void const *pvParameters){
	
	// ensure that INS_task got already executed at least one time
	while (ins_correct_angle[0] == 0 && ins_correct_angle[1] == 0 && ins_correct_angle[2] == 0);
	
	while(1)
	{
		if (new_robot_control_mode != robot_control_mode)
		{
			robot_control_mode = new_robot_control_mode;
		}
		
		switch (robot_control_mode){
			
			case REMOTE_CONTROLLER_MODE:
			case CHASSIS_FOLLOW_GIMBAL_PC_MODE:
			case FIXED_GIMBAL_PC_MODE:
				gimbal_control_loop(robot_control_mode);
			break;
			
			case ERROR_STOP_MODE:
				CAN_cmd_gimbal((int16_t) 0, (int16_t) 0);
			break;
		}
		
		osDelay(1);
	}
}
