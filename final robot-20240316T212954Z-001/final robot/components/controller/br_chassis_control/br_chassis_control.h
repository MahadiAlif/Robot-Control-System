#ifndef BR_CHASSIS_CONTROL_H
#define BR_CHASSIS_CONTROL_H

#include <stdint.h>

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/* flags to set manually */
#define IS_90DEG_MODE_ENABLED 							1		// set to 1 if you want to enable the 90 degree mode of balancing robot chassis, 0 otherwise
#define IS_CHASSIS_ROT_MODE_ENABLED					0		// set to 1 if you want to enable the contiguous chassis rotation in the balancing robot, 0 otherwise
#define IS_UNSTUCK_FROM_WALL_MODE_ENABLED 	1		// set to 1 if you want to enable the unstuck from wall mode of balancing robot chassis, 0 otherwise

/* fixed values */
#define NUM_BR_WHEELS 2			// number of wheels of balancing robot chassis

/* values to be tuned */
#define MAX_BR_LQR_THETA_OUT 		1		// max ouput of LQR controller that, given current angular position and velocity of balancing robot, returns a reference for the linear position of wheels

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/* balancing robot parameters */
typedef struct balancing_robot_model_s
{
	float r;				// wheel radius (m)
	float w;				// width (i.e. distance between the 2 wheels of the robot)
	
	// output measurements array
	float y[6];
	
	// state estimation
	float state_estim[6];
	float state_estim_prev[6];
	
	// reference signals
	float ref[6];
	float ref_prev[6];
	float pos_L_to_mantain_on_ground;
	float pos_R_to_mantain_on_ground;
	
	// control signals
	float control_signals[2];		// control signals to left and right wheel
	
	// errors w.r.t. reference signals
	float error[6];
	float error_prev[6];
	float integral_error[6];
	
	// angle between chassis and gimbal
	float chassis_gimbal_relative_yaw_angle;

} balancing_robot_model_t;


/*** model ***/
extern struct robot br_chassis;		// TODO remove this and in gimbal_control.c add another element in state vector x that contains the same info as br_chassis.x[6] (chassis-gimbal relative angle in radiants)

/*** control ***/
extern float min_pos_dot_L_to_trigger_BR_repositioning;		// in meters
extern float min_pos_dot_R_to_trigger_BR_repositioning;		// in meters
extern float min_theta_to_trigger_BR_repositioning;				// in rad/s

/*** gains to be tuned ***/


/*** flags ***/



/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

/************************************************************************************************************************
	NAME: br_chassis_control_loop
	
	DESCRIPTION:
	- implements the feedback-control loop for the chassis of balancing robot
	- main operations:
		1. acquires the state feedback from sensors
		2. computes the control signals
		3. sends the control signals to the motors of the robot
************************************************************************************************************************/
void br_chassis_control_loop();

#endif
