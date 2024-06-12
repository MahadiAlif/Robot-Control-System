#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "std_chassis_control.h"
#include "control_util.h"
#include "robot_config.h"
#include "referee_alg.h"
#include "remote_control.h"
#include "control_alg.h"
#include "CAN_receive.h"
#include "bmi088driver.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "math_util.h"
#include "chassis_task.h"


/**************************************************************
	NOTES about the Standard Chassis:
	- The CAN IDs of the wheels are: 1->front-left wheel, 2->front-right wheel, 3->back-right wheel, 4->back-left wheel
	- The signs of the "forward" control signals to the wheels are: 1 -> +, 2 -> -, 3 -> -, 4 -> +
**************************************************************/

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** model ***/
static struct robot std_chassis;
static uint8_t n = NUM_STD_WHEELS + 1;		// number of robot states (4 wheels + 1 chassis/gimbal relative angle)
static uint8_t m = NUM_STD_WHEELS;				// number of robot inputs

/*** control ***/
static struct PID_ pid[NUM_STD_WHEELS];		// PID controllers for each standard robot wheel
static float Kp = 1.2;
static float Ki = 1.0;
static float Kd = 0.0;

/*** gains to be tuned ***/
static float gain_overall_u = 500;				// gain of the overall control signal (before sending it to the motors)
static float gain_decr_speed = 0.6;				// gain that decreases wheels angular speed (when certain conditions are met)

/*** references ***/
static float max_ref_wheels_lin = 20.0; 		// max reference to wheels angular speed when chassis is NOT rotating 	[rad/s]
static float max_ref_wheels_rot = 28.3; 		// max reference to wheels angular speed when chassis is rotating				[rad/s]
static float ref_wheels_chassis_contig_rot = 18.0;		// 18					// reference to wheels angular speed when chassis contiguously rotates	[rad/s]
static float max_ref_wheels_chassis_follow_gimbal = 30.0;	// reference to wheels angular speed for chassis to follow the gimbal		[rad/s]

/*** sentry autonomous movements ***/
#if IS_SENTRY
static float coeff_patrol_ref_chassis_contig_rot = 0.7;						// coefficient to make faster/slower the chassis contiguous rotation while sentry is in patrol mode 		[percentage]
static int8_t sign_chassis_contig_rot = 1;												// sign of sentry's chassis rotation (+1 means counter-clockwise; -1 means clockwise)
static float time_ms_last_switch_sign_chassis_contig_rot = 0;			// timestamp of the last time that the chassis' rotation sign was switched	[ms]
static float time_switch_sign_chassis_contig_rot = 10.0;					// time that elapses before the chassis' rotation sign is changed (from +1 to -1, or vice versa)		[s]
#endif

/*** flags ***/
static int is_first_iter = 1;		// true if the current algorithm iteration is the first one



/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void std_chassis_control_loop() {
	
	if (is_first_iter) {
		
		robot_init(&std_chassis, n, m);			// initialize the standard robot chassis data structure
		
		for (int wheel = 0; wheel < NUM_STD_WHEELS; wheel++)
			PID_init_(&pid[wheel], Kp, Ki, Kd, dt_chassis);
		
#if IS_POW_CONS_CHASSIS_ENABLED
		chassis_power_consumption_init();
#endif
	}
	
	/* take measurements from wheels' sensors */
	for (int wheel = 0; wheel < NUM_STD_WHEELS; wheel++) {
		std_chassis.x[wheel] = get_chassis_motor_measures(wheel)->ang_vel_radsec;
	}
	
	/* take relative Yaw position of chassis w.r.t. gimbal */
	std_chassis.x[4] = (get_yaw_gimbal_motor_measures()->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET);
	
	// bring the chassis-gimbal relative yaw position in range [-pi,+pi]
	std_chassis.x[4] = nearest_contig_ref_angle(std_chassis.x[4], 0.0);
	
	
	/* take references from keyboard/joystick */
	float ref_fwd_bwd;						// references received from external commands for FORWARD/BACKWARD movements
	float ref_left_right;					// references received from external commands for LEFT/RIGHT movements
	float ref_fwd_bwd_fl_br;				// reference to wheels from remote controller for forward/backward movement of front-left and back-right wheels [rad/s]
	float ref_fwd_bwd_fr_bl;				// reference to wheels from remote controller for forward/backward movement of front-right and back-left wheels [rad/s]
	float ref_left_right_fl_br;		// reference to wheels from remote controller for left/right of front-left and back-right wheels [rad/s]
	float ref_left_right_fr_bl;		// reference to wheels from remote controller for left/right of front-right and back-left wheels [rad/s]
	float ref_chassis_rot;						// reference to wheels for the rotation of the chassis w.r.t. the gimbal [rad/s]
	

#if IS_STD

	if (switch_is_up(rc_right_switch)) {			// REMOTE CONTROLLER COMMANDS
		
		// take references from remote controller
		ref_fwd_bwd 		= (float) get_remote_control_point()->rc.ch[3];
		ref_left_right 	= (float) get_remote_control_point()->rc.ch[2];
		
		if (switch_is_mid(rc_left_switch)) {			// CHASSIS CONTIGUOUS ROTATION
			
			ref_fwd_bwd_fl_br 		= (ref_fwd_bwd / MAX_RC_TILT) 	 * max_ref_wheels_rot * sin(- std_chassis.x[4] + pi/4);
			ref_fwd_bwd_fr_bl 		= (ref_fwd_bwd / MAX_RC_TILT) 	 * max_ref_wheels_rot * cos(- std_chassis.x[4] + pi/4);
			ref_left_right_fl_br 	= (ref_left_right / MAX_RC_TILT) * max_ref_wheels_rot * sin(+ std_chassis.x[4] + pi/4);
			ref_left_right_fr_bl 	= (ref_left_right / MAX_RC_TILT) * max_ref_wheels_rot * cos(+ std_chassis.x[4] + pi/4);
			
			ref_chassis_rot = ref_wheels_chassis_contig_rot;
			
		}
		else {		// CHASSIS-FOLLOW-GIMBAL
			
			// take forward/backward and right/left references
			ref_fwd_bwd_fl_br 		= (ref_fwd_bwd / MAX_RC_TILT) * max_ref_wheels_lin;
			ref_fwd_bwd_fr_bl 		= ref_fwd_bwd_fl_br;
			ref_left_right_fl_br 	= (ref_left_right / MAX_RC_TILT) * max_ref_wheels_lin;
			ref_left_right_fr_bl 	= ref_left_right_fl_br;
			
			// compute reference for chassis-gimbal alignment
			ref_chassis_rot = max(min(std_chassis.x[4], pi/2), -pi/2) * (2/pi) * max_ref_wheels_chassis_follow_gimbal;
		}
	
	}
	else if (switch_is_mid(rc_right_switch)) {			// KEYBOARD/MOUSE COMMANDS
		
		// take references from keyboard/mouse
		compute_weights_WASD_keys(dt_chassis);
		
		if (switch_is_mid(rc_left_switch) || is_key_pressed(KEY_SHIFT)) {				// CHASSIS CONTIGUOUS ROTATION
		
			// forward/backward wheels 1, 2, 3, 4
			ref_fwd_bwd_fl_br = 0.0;
			ref_fwd_bwd_fr_bl = 0.0;
			ref_fwd_bwd_fl_br += max_ref_wheels_rot * sin(- std_chassis.x[4] + pi/4) * weight_fwd_key;
			ref_fwd_bwd_fr_bl += max_ref_wheels_rot * cos(- std_chassis.x[4] + pi/4) * weight_fwd_key;	
			ref_fwd_bwd_fl_br -= max_ref_wheels_rot * sin(- std_chassis.x[4] + pi/4) * weight_bwd_key;
			ref_fwd_bwd_fr_bl -= max_ref_wheels_rot * cos(- std_chassis.x[4] + pi/4) * weight_bwd_key;
			
			// right/left wheels 1, 2, 3, 4
			ref_left_right_fl_br = 0.0;
			ref_left_right_fr_bl = 0.0;
			ref_left_right_fl_br += max_ref_wheels_rot * sin(std_chassis.x[4] + pi/4) * weight_right_key;
			ref_left_right_fr_bl += max_ref_wheels_rot * cos(std_chassis.x[4] + pi/4) * weight_right_key;
			ref_left_right_fl_br -= max_ref_wheels_rot * sin(std_chassis.x[4] + pi/4) * weight_left_key;
			ref_left_right_fr_bl -= max_ref_wheels_rot * cos(std_chassis.x[4] + pi/4) * weight_left_key;
			
			ref_chassis_rot = ref_wheels_chassis_contig_rot;
			
		}
		else {				// CHASSIS-FOLLOW-GIMBAL
			
			// forward/backward wheels 1 and 3
			ref_fwd_bwd_fl_br = 0.0;
			ref_fwd_bwd_fl_br += max_ref_wheels_lin * weight_fwd_key;
			ref_fwd_bwd_fl_br -= max_ref_wheels_lin * weight_bwd_key;
			// forward/backward wheels 2 and 4
			ref_fwd_bwd_fr_bl = ref_fwd_bwd_fl_br;
			
			// right/left wheels 1 and 3
			ref_left_right_fl_br = 0.0;
			ref_left_right_fl_br += max_ref_wheels_lin * weight_right_key;
			ref_left_right_fl_br -= max_ref_wheels_lin * weight_left_key;
			// right/left wheels 2 and 4
			ref_left_right_fr_bl = ref_left_right_fl_br;
			
			// compute reference for chassis-gimbal alignment
			ref_chassis_rot = max(min(std_chassis.x[4], pi/2), -pi/2) * (2/pi) * max_ref_wheels_chassis_follow_gimbal;
		}
		
		
		// if SHIFT is pressed, modify wheels reference speed in order to go slower/faster than usual
		if (is_key_pressed(KEY_CTRL)) {
			
			ref_fwd_bwd_fl_br *= gain_decr_speed;
			ref_fwd_bwd_fr_bl *= gain_decr_speed;
			ref_left_right_fl_br *= gain_decr_speed;
			ref_left_right_fr_bl *= gain_decr_speed;
		}
	}
	
#elif IS_SENTRY

	if (switch_is_up(rc_right_switch)) {			// REMOTE CONTROLLER COMMANDS
		
		// take references from remote controller
		ref_fwd_bwd 		= (float) get_remote_control_point()->rc.ch[3];
		ref_left_right 	= (float) get_remote_control_point()->rc.ch[2];
		
		if (switch_is_mid(rc_left_switch)) {			// CHASSIS CONTIGUOUS ROTATION
			
			ref_fwd_bwd_fl_br 		= (ref_fwd_bwd / MAX_RC_TILT) 	 * max_ref_wheels_rot * sin(- std_chassis.x[4] + pi/4);
			ref_fwd_bwd_fr_bl 		= (ref_fwd_bwd / MAX_RC_TILT) 	 * max_ref_wheels_rot * cos(- std_chassis.x[4] + pi/4);
			ref_left_right_fl_br 	= (ref_left_right / MAX_RC_TILT) * max_ref_wheels_rot * sin(+ std_chassis.x[4] + pi/4);
			ref_left_right_fr_bl 	= (ref_left_right / MAX_RC_TILT) * max_ref_wheels_rot * cos(+ std_chassis.x[4] + pi/4);
			
			ref_chassis_rot = ref_wheels_chassis_contig_rot;
			
		}
		else {		// CHASSIS-FOLLOW-GIMBAL
			
			// take forward/backward and right/left references
			ref_fwd_bwd_fl_br 		= (ref_fwd_bwd / MAX_RC_TILT) * max_ref_wheels_lin;
			ref_fwd_bwd_fr_bl 		= ref_fwd_bwd_fl_br;
			ref_left_right_fl_br 	= (ref_left_right / MAX_RC_TILT) * max_ref_wheels_lin;
			ref_left_right_fr_bl 	= ref_left_right_fl_br;
			
			// compute reference for chassis-gimbal alignment
			ref_chassis_rot = max(min(std_chassis.x[4], pi/2), -pi/2) * (2/pi) * max_ref_wheels_chassis_follow_gimbal;
		}
	
	}
	else if (switch_is_mid(rc_right_switch)) {			// FULLY AUTONOMOUS
		
		// if needed, switch the chassis' rotation sign
		if (HAL_GetTick() - time_ms_last_switch_sign_chassis_contig_rot >= time_switch_sign_chassis_contig_rot*1e3) {
			
			sign_chassis_contig_rot *= -1;		// toggle the rotation sign
			time_ms_last_switch_sign_chassis_contig_rot = HAL_GetTick();			// save current timestamp (for next sign toggle)
		}
		
		/* sentry doesn't move around the battlefield */
		ref_fwd_bwd_fl_br = 0;
		ref_fwd_bwd_fr_bl = 0;
		ref_left_right_fl_br = 0;
		ref_left_right_fr_bl = 0;
		
		/* setry's chassis contiguous rotation */
		ref_chassis_rot = ref_wheels_chassis_contig_rot * sign_chassis_contig_rot;
		
//		if (sentry_state == PATROL)
//			ref_chassis_rot *= coeff_patrol_ref_chassis_contig_rot;
		
#if !IS_SENTRY_CHASSIS_ROT_ENABLED
		ref_chassis_rot = 0;
#endif
	}
	
#endif

	
	
	
	/* compute the references for the wheels speed */
	std_chassis.ref[0] = + ref_fwd_bwd_fl_br + ref_left_right_fl_br - ref_chassis_rot;
	std_chassis.ref[1] = - ref_fwd_bwd_fr_bl + ref_left_right_fr_bl - ref_chassis_rot;
	std_chassis.ref[2] = - ref_fwd_bwd_fl_br - ref_left_right_fl_br - ref_chassis_rot;
	std_chassis.ref[3] = + ref_fwd_bwd_fr_bl - ref_left_right_fr_bl - ref_chassis_rot;
	
	
	/* compute the control signals for the 4 wheels of standard chassis */;
	ref_error(&std_chassis, dt_chassis);
	
	for (int wheel = 0; wheel < NUM_STD_WHEELS; wheel++) {
		
		// reset the integral error area if the reference have been overshot
		reset_integr_error_if_overshoot(&std_chassis, wheel);
		
		// perform PID control on each wheel
		std_chassis.u[wheel] = PID_control(	&pid[wheel],
																				std_chassis.error[wheel],	std_chassis.integr_error[wheel],	std_chassis.deriv_error[wheel],
																				UNDEF,										UNDEF,														UNDEF,
																				NULL,											NULL,															NULL);
		
		// overall gain for standard chassis wheels
		std_chassis.u[wheel] *= gain_overall_u;
		
		// saturation of the control signals (in order to avoid possible damages)
		saturate(&std_chassis.u[wheel], STD_CHASSIS_U_SAT);
	}
	
	
	// check chassis power consumption limit
#if IS_POW_CONS_CHASSIS_ENABLED
		check_chassis_power_consumption();
#endif
	
	
	if (IS_CHASSIS_ENABLED)
		CAN_cmd_chassis((int16_t) std_chassis.u[0],
										(int16_t) std_chassis.u[1],
										(int16_t) std_chassis.u[2],
										(int16_t) std_chassis.u[3]);
	
	/* save the robot state of the current iteration, in order to use it for the next iteration */
	save_prev_robot_state(&std_chassis);
	
	// first algorithm iteration terminates here
	is_first_iter = 0;
}



