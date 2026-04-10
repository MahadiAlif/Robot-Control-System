#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include <stdint.h>


/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

#define TAKE_PITCH_FROM_MOTOR_ENCODER 0


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
extern gimbal_model_t standard_gimbal;

/*** control ***/
extern float T_gimbal; 		// inverse of the frequency of gimbal controller (1000 Hz)
extern float gimbal_yaw_LQR_K[2];		// LQR gain of the standard robot's gimbal yaw
extern float gimbal_pitch_LQR_K[2];		// LQR gain of the standard robot's gimbal pitch
extern float standard_gimbal_I_gain[2];	// integral gain for error w.r.t. gimbal's position
extern float current_middle_yaw;
extern float current_middle_pitch;
extern float ref_gimbal_yaw;			// reference for Yaw
extern float ref_gimbal_pitch;		// reference for Pitch
extern float cmdYawGimbal;			// Yaw command sent by the joystick
extern float cmdPitchGimbal;		// Pitch command sent by the joystick
extern float cmdYawGimbal_prev;
extern float cmdPitchGimbal_prev;
extern float saturation_control_signal_yaw_gimbal;			// Yaw control signal saturation value
extern float saturation_control_signal_pitch_gimbal;		// Pitch control signal saturation value
extern float max_gimbal_pitch_inclination;

/*** gains to be tuned ***/
// gains (to be tuned manually) that should be such that the motors actually apply a torque equal to the desired one
extern float gain_torque_to_control_signal_yaw;
extern float gain_torque_to_control_signal_pitch;
// LQR Yaw and Pitch gains
extern float gain_LQR_control_signal_gimbal_yaw;
extern float gain_LQR_control_signal_gimbal_pitch;
// gains to mantain a certain orientation of the gimbal
extern float gain_mantain_position_yaw;
extern float gain_mantain_position_pitch;
// gain to compensate the fact that the center of mass of the gimbal is inclined backward
extern float gain_pitch_goes_down;

/*** exponential gain values ***/
// maximum values (for Yaw and Pitch) of the gains that, the more Yaw and Pitch are near to their reference, the more is large (minimum gain is fixed to 1)
extern float max_exp_yaw_error_gain;
extern float max_exp_pitch_error_gain;
// Yaw and Pitch angles at which the exponential gain is = 1
extern float max_exp_gain_yaw_range;
extern float max_exp_gain_pitch_range;
// points between 0 and "max_exp_gain_yaw_range"/"max_exp_gain_pitch_range" that determines the exponent of the exponential gain
extern float middle_angle_exp_yaw;
extern float middle_angle_exp_pitch;
// values of the exponential gain corresponding to "middle_angle_exp_yaw" and "middle_angle_exp_pitch"
extern float middle_value_exp_yaw;
extern float middle_value_exp_pitch;

/*** gimbal mantain position values ***/
extern float yaw_angle_where_mantain_position_starts;		// x1 for yaw
extern float pitch_angle_where_mantain_position_starts;	// x1 for pitch
extern float length_yaw_gain_increase;									// x4-x1 for yaw
extern float length_pitch_gain_increase;								// x4-x1 for pitch
extern float max_gain_mantain_yaw;											// ymax for yaw
extern float max_gain_mantain_pitch;										// ymax for pitch
extern float epsilon_yaw;																// in (x1 + (x1+x4)/4) the gain is 1+epsilon 
extern float epsilon_pitch;															// in (x1 + (x1+x4)/4) the gain is 1+epsilon

/*** gimbal yaw and pitch integral values ***/
extern float gain_integral_yaw_error;
extern float gain_integral_pitch_error;
extern float range_where_trigger_yaw_integral;
extern float range_where_trigger_pitch_integral;

/*** gimbal mouse movement values ***/
extern float mouse_meters_yaw_rad_ratio;		// if mouse travels 0.01 meters, then yaw has to travel 35*pi/180 rad
extern float mouse_meters_pitch_rad_ratio;		// if mouse travels 0.005 meters, then yaw has to travel 30*pi/180 rad
extern uint32_t clock_ms_mouse_gimbal;
extern uint32_t clock_ms_mouse_gimbal_prev;

/*** sentry values to scan environment and search enemies ***/
extern float sentry_yaw_freq_scan_environment;
extern float sentry_pitch_freq_scan_environment;
extern float sentry_yaw_amplitude_scan_environment;
extern float sentry_pitch_amplitude_scan_environment;

/*** flags ***/
extern int need_to_initialize_gimbal;
extern int need_to_mantain_gimbal_yaw_position;
extern int need_to_mantain_gimbal_pitch_position;
extern int need_to_mantain_gimbal_yaw_position_prev;
extern int need_to_mantain_gimbal_pitch_position_prev;
extern int need_to_set_middle_yaw;
extern int need_to_set_middle_pitch;
extern int need_to_set_gain_pitch;
extern int need_to_set_initial_mouse_gimbal_position;
extern int pitch_is_going_down;	// check if pitch of gimbal if going either down or up (useful to compute the balance angle of BR at runtime)


// TODO delete this (when the code will be refactored)
extern float sat_ei_yaw;
extern float sat_ei_pitch;

/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void gimbal_control_loop(uint8_t robot_control_mode);
void gimbal_control_init(void);



#endif



