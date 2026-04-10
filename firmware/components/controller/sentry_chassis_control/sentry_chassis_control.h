#ifndef SENTRY_CHASSIS_CONTROL_H
#define SENTRY_CHASSIS_CONTROL_H

#include <stdint.h>

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

typedef struct sentry_chassis_model_s {

	// physical parameters
	float wheel_radius;
	
	// measurements
	float wheel_angular_speed;		// rad/s
	float lin_pos_on_track;				// meters
	float lin_pos_on_track_prev;	// meters
	
	// references
	float ref_wheel_angular_speed;
	float ref_wheel_angular_speed_prev;
	float ref_lin_pos_on_track;
	float ref_lin_pos_on_track_prev;
	
	// control signals
	float control_signal;
	
	// errors w.r.t. references
	float error;
	float integral_error;
	
} sentry_chassis_model_t;
	


/*** model ***/
extern sentry_chassis_model_t sentry_chassis;		// model of the sentry robot's chassis

/*** control ***/
extern float T_sentry_chassis; 		// inverse of the frequency of sentry chassis controller (1000 Hz)
extern float sentry_chassis_P_gain;		// proportional gain of the sentry robot's chassis (for PID)
extern float sentry_chassis_I_gain;		// integral gain of the sentry robot's chassis (for PID)
extern float max_random_mov_right_left;	// range within which the sentry's chassis can do a random movement(in meters)
extern float lin_speed_along_track_without_enemies;	// constant linear velocity that sentry will have along the track when there're no enemies detected (in m/s)
extern float lin_speed_along_track_with_enemies;	// constant linear velocity that sentry will have along the track when there's at least one enemy detected (in m/s)

/*** gains to be tuned ***/
extern float gain_sentry_chassis_wheel;

/*** flags ***/
extern int need_to_initialize_sentry_chassis;
extern int enemy_detected;		// 1 if at least one enemy has been detected, otherwise 0
extern int mov_direction_along_track;	// this is either +1 or -1, and indicates in which direction the chassis needs to move along the track
extern int enemy_just_detected;


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void sentry_chassis_control_loop(uint8_t robot_control_mode);
void sentry_chassis_control_init(void);

#endif



