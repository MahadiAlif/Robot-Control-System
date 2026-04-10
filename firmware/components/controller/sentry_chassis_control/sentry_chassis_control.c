#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "sentry_chassis_control.h"
#include "control_util.h"
#include "robot_config.h"
#include "referee_alg.h"
#include "control_alg.h"
#include "CAN_receive.h"
#include "bmi088driver.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "math_util.h"
#include "remote_control.h"


/**************************************************************
Notes about the Sentry Chassis:
- ...
- ...
**************************************************************/

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** model ***/
sentry_chassis_model_t sentry_chassis;		// model of the sentry robot's chassis

/*** control ***/
float T_sentry_chassis = 0.001; 		// inverse of the frequency of sentry chassis controller (1000 Hz)
float sentry_chassis_P_gain;		// proportional gain of the sentry robot's chassis (for PID)
float sentry_chassis_I_gain;		// integral gain of the sentry robot's chassis (for PID)
float max_random_mov_right_left = 1.0;	// range within which the sentry's chassis can do a random movement(in meters)
float lin_speed_along_track_without_enemies = 0.5;	// constant linear velocity that sentry will have along the track when there're no enemies detected (in m/s)
float lin_speed_along_track_with_enemies = 0.7;	// constant linear velocity that sentry will have along the track when there's at least one enemy detected (in m/s)

/*** gains to be tuned ***/
float gain_sentry_chassis_wheel = 0; //500;

/*** flags ***/
int need_to_initialize_sentry_chassis = 1;
int enemy_detected = 0;		// 1 if at least one enemy has been detected, otherwise 0
int mov_direction_along_track = 1;	// this is either +1 or -1, and indicates in which direction the chassis needs to move along the track
int enemy_just_detected = 1;


int first_sentry_iteration = 1;


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void sentry_chassis_control_loop(uint8_t robot_control_mode) {
	
	if (need_to_initialize_sentry_chassis) {
		
		sentry_chassis_control_init();
		need_to_initialize_sentry_chassis = 0;
	}
	
	/* take measurements from wheel sensors */
	sentry_chassis.lin_pos_on_track = get_chassis_motor_measures(0)->contig_ang_pos_rad * sentry_chassis.wheel_radius;
	sentry_chassis.wheel_angular_speed = get_chassis_motor_measures(0)->ang_vel_radsec;
	
	/* check if there're enemies in front of the sentry (performed in gimbal_control.c, because it get executed more frequently) */
	
	/* if enemy_detected == 1 then do small random movements, otherwise go forward and backward on the track searching for enemies */
	if (/*enemy_detected*/ 1) {
		
//		if (enemy_just_detected) {
//			sentry_chassis.lin_pos_on_track = sentry_chassis.ref_lin_pos_on_track;	// this is used to trigger
//			enemy_just_detected = 0;
//		}
		if (first_sentry_iteration) {
			sentry_chassis.ref_lin_pos_on_track = sentry_chassis.lin_pos_on_track;
			first_sentry_iteration = 0;
		}
		
		if ((mov_direction_along_track == 1 && sentry_chassis.lin_pos_on_track >= sentry_chassis.ref_lin_pos_on_track) ||
				(mov_direction_along_track == -1 && sentry_chassis.lin_pos_on_track <= sentry_chassis.ref_lin_pos_on_track)) {
			
			// generate new random reference on track
			sentry_chassis.ref_lin_pos_on_track = sentry_chassis.lin_pos_on_track + ((float)rand()/(float)RAND_MAX - 0.5)*max_random_mov_right_left;
			sentry_chassis.ref_lin_pos_on_track = min(max(sentry_chassis.ref_lin_pos_on_track, 0.0), MAX_TRACK_LENGTH);
			
			// determine in which direction we need to go on the track
			if (sentry_chassis.ref_lin_pos_on_track >= sentry_chassis.lin_pos_on_track)
				mov_direction_along_track = 1;
			else
				mov_direction_along_track = -1;
		}
		
		// set references
		sentry_chassis.ref_wheel_angular_speed = (lin_speed_along_track_with_enemies / sentry_chassis.wheel_radius) * mov_direction_along_track;
		
	}
	else {
		
		// prepare for next time that an enemy will be detected
		if (!enemy_just_detected)
			enemy_just_detected = 1;
		
		// check in which direction we have to move along the track
		if (mov_direction_along_track == 1 && sentry_chassis.lin_pos_on_track >= MAX_TRACK_LENGTH)
			mov_direction_along_track = -1;
		else if (mov_direction_along_track == -1 && sentry_chassis.lin_pos_on_track <= 0)
			mov_direction_along_track = 1;
		
		// set references
		if (mov_direction_along_track == 1) {
			sentry_chassis.ref_lin_pos_on_track = MAX_TRACK_LENGTH;
			sentry_chassis.ref_wheel_angular_speed = lin_speed_along_track_without_enemies / sentry_chassis.wheel_radius;
		}
		else if (mov_direction_along_track == -1) {
			sentry_chassis.ref_lin_pos_on_track = 0;
			sentry_chassis.ref_wheel_angular_speed = - lin_speed_along_track_without_enemies / sentry_chassis.wheel_radius;
		}
	}
	
	/* compute the control signal for the wheel of sentry's chassis */
	PI_controller(STANDARD_CHASSIS);
	
	/* overall gain for sentry chassis wheels */
	sentry_chassis.control_signal *= gain_sentry_chassis_wheel;
	
	// saturation of the control signal (in order to avoid possible damages)
	saturate(&sentry_chassis.control_signal, MAX_SENTRY_CHASSIS_WHEELS_CONTROL_SIGNAL_AMPLITUDE);
	
	if (USE_REFEREE_SYS_CHASSIS_POWER_CONSUMPTION)
		check_chassis_power_consumption();
	
	// send the control signal
	if (IS_CHASSIS_ENABLED)
		CAN_cmd_chassis((int16_t) sentry_chassis.control_signal, (int16_t) 0, (int16_t) 0, (int16_t) 0);
	
	// set previous values for next iteration
	sentry_chassis.ref_wheel_angular_speed_prev = sentry_chassis.ref_wheel_angular_speed;
	sentry_chassis.ref_lin_pos_on_track_prev = sentry_chassis.ref_lin_pos_on_track;
	sentry_chassis.lin_pos_on_track_prev = sentry_chassis.lin_pos_on_track;
}


void sentry_chassis_control_init(void) {
	
	sentry_chassis_model_init(&sentry_chassis);			// initialization of the model for the sentry chassis
	PI_gain_init(STANDARD_CHASSIS);
}



