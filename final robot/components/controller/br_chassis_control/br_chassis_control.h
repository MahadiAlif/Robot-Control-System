#ifndef BR_CHASSIS_CONTROL_H
#define BR_CHASSIS_CONTROL_H

#include <stdint.h>

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/* flags to set manually */
#define IS_90DEG_MODE_ENABLED 							1		// set to 1 if you want to enable the 90 degree mode of balancing robot chassis, 0 otherwise
#define IS_CHASSIS_ROT_MODE_ENABLED					1		// set to 1 if you want to enable the contiguous chassis rotation in the balancing robot, 0 otherwise
#define IS_UNSTUCK_FROM_WALL_MODE_ENABLED 	1		// set to 1 if you want to enable the unstuck from wall mode of balancing robot chassis, 0 otherwise

/* fixed values */
#define NUM_BR_WHEELS 2			// number of wheels of balancing robot chassis

/* values to be tuned */
#define MAX_BR_POS_LQR_THETA_OUT 								1				// max ouput of LQR controller that, given current angular position and velocity of balancing robot, returns a reference for the linear position of wheels
#define BR_TIME_RAISING_EDGE_WASD_KEYS					3.0			// time needed for the keyboard's weights to go from 0 to 1 when a WASD key is pressed		[s]
#define BR_TIME_FALLING_EDGE_BRAKE_WASD_KEYS		1.0			// time needed for the keyboard's weights to go from 0 to 1 when a WASD key is pressed in the OPPOSITE DIRECTION w.r.t. the current COM linear velocity		[s]
#define BR_TIME_FALLING_EDGE_WASD_KEYS					2.0			// time needed for the keyboard's weights to go from 1 to 0 when a WASD key is released (i.e., NOT pressed anymore)			[s]
#define STD_TIME_RAISING_EDGE_WASD_KEYS					1.0			// time needed for the keyboard's weights to go from 0 to 1 when a WASD key is pressed		[s]
#define STD_TIME_FALLING_EDGE_BRAKE_WASD_KEYS		0.2			// time needed for the keyboard's weights to go from 0 to 1 when a WASD key is pressed in the OPPOSITE DIRECTION w.r.t. the current COM linear velocity		[s]
#define STD_TIME_FALLING_EDGE_WASD_KEYS					0.3			// time needed for the keyboard's weights to go from 1 to 0 when a WASD key is released (i.e., NOT pressed anymore)			[s]


/*
NOTES ABOUT THE ABOVE MACROS:
- BR_TIME_RAISING_EDGE_WASD_KEYS and BR_TIME_FALLING_EDGE_WASD_KEYS can be any float value STRICTLY > 0.
*/

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
