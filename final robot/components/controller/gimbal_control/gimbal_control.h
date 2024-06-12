#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdint.h>


/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

#define TAKE_PITCH_FROM_MOTOR_ENCODER			0
#define ENABLE_CV_AUTO_AIM_PITCH_PRED			1			// 1 if (from the enemy's coordinates obtained by the CV detection) we compute a pitch reference that take into account the gravity force on bullets; 0 otherwise (just point towards the enemy).

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/* gimbal parameters */
typedef struct gimbal_model_s {
	
	// output measurements
	float y[4];
	
	// state estimation
	float state_estim[4];
	float state_estim_prev[4];
	
	// reference signals
	float ref[4];
	float ref_prev[4];
	
	// control signals
	float control_signals[2];		// Yaw and Pitch control signals for gimbal
	
	// errors w.r.t. reference signals
	float error[4];
	float error_prev[4];
	float integral_error[2];	// yaw and pitch angles integral errors
	
} gimbal_model_t;


/*** model ***/
extern struct robot gimbal;


/*** CV-driven gimbal ***/
// gimbal auto-aim structure
extern struct cv_gimbal_auto_aim cv_gimbal;

/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void gimbal_control_loop(void);



#endif



