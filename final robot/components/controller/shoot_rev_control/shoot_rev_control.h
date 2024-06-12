#ifndef SHOOT_REV_CONTROL_H
#define SHOOT_REV_CONTROL_H

#include <stdint.h>

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

typedef struct shoot_motors_model_s {
	
	/* NOTE: sentry robot has 4 shoot motors, while other robots have only 2 */
	
	// left and right shoot motors measured speed
	float y[4];
	
	// left and right shoot motors estimated speed
	float speed_radsec[4];
	
	// reference signals
	float ref[4];		// reference for left and right shoot motors speed
	float ref_prev[4];
	
	// control signals
	float control_signals[4];		// control signals for left and right shoot motors
	
	// errors w.r.t. reference signals (speed of L and R motor)
	float error[4];
	float error_prev[4];
	float integral_error[4];
	
} shoot_motors_model_t;


typedef struct rev_motor_model_s {
	
	// rev motor's output
	float y[2];
	
	// rev motor's state
	float state_estim[2];			// rev motor's contiguous position and speed
	
	// reference signals
	float ref[2];		// reference for position and speed of rev motor
	float ref_prev[2];
	
	// control signal
	float control_signal;		// control signal for rev motor
	
	// errors w.r.t. reference signals (position and velocity of rev motor)
	float error[2];
	float error_prev[2];
	float integral_error[2];
	
} rev_motor_model_t;


/*** model ***/
extern shoot_motors_model_t shoot_motors;		// model of the left and right shoot motor
extern rev_motor_model_t rev_motor;					// model of the rev motor

/*** control ***/
extern float T_shoot;
extern float T_rev;
extern float shoot_P_gain[4];		// proportional gain of the shoot motor (for PID)
extern float shoot_I_gain[4];		// integral gain of the shoot motor (for PID)
extern float rev_LQR_K[2];			// LQR gain of the standard robot's rev motor
extern float rev_I_gain[2];			// integral gain for error w.r.t. rev motor's position
extern float ref_shoot_motors_speed_radsec;				// needs to be set by the keyboard (this is just the initial value)
extern float initial_shoot_speed_when_restart;		// initial speed of shoot motors when we activate either SINGLE_SHOOT_MODE or MULTIPLE_SHOOT_MODE
extern float saturation_control_signal_shoot_L;		// left shoot motor control signal saturation value
extern float saturation_control_signal_shoot_R;		// right shoot motor control signal saturation value
extern int rev_rotation_mode;
extern int enable_shoot_one_bullet;
extern float error_threshold_to_shoot_new_sigle_bullet;
extern float error_threshold_to_shoot_new_multiple_bullet;
extern uint32_t start_clock_to_prevent_rev_unlock_indefinitely;
extern float cv_shoot_freq;																// shooting frequency for sentry robot (determined by how far is the enemy robot detected by the CV)

/*** gains to be tuned ***/
// shoot (left and right) and rev motors gains
extern float gain_LQR_control_signal_shoot_L;
extern float gain_LQR_control_signal_shoot_R;
extern float gain_LQR_control_signal_rev;

/*** rev motor integral values ***/
extern float gain_integral_rev_error;
extern float range_where_trigger_rev_integral;

/*** flags ***/
extern int need_to_initialize_shoot_motor;
extern int need_to_initialize_rev_motor;
extern int allowed_to_shoot_single_bullet;	// 1 if we want to shoot and we actually can
extern int want_to_shoot_single_bullet;			// 1 if we want to shoot
extern int need_to_unlock_rev_motor;
extern int need_to_set_unlock_rev_motor_ref;





/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void shoot_rev_control_loop(uint8_t shooting_control_mode, uint8_t shooting_control_mode_prev);

void shoot_control(uint8_t shooting_control_mode, uint8_t shooting_control_mode_prev);
void shoot_control_init(void);

void rev_control(uint8_t shooting_control_mode, uint8_t shooting_control_mode_prev);
void rev_control_init(void);


#endif




