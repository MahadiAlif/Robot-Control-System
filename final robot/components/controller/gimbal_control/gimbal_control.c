#include "gimbal_control.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "control_alg.h"
#include "control_util.h"
#include "CAN_receive.h"
#include "bmi088driver.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "math_util.h"
#include "remote_control.h"
#include "AI_receive.h"
#include "robot_config.h"
#include "gimbal_task.h"


/**************************************************************
Notes about the Gimbal:
- The board is placed on the gimbal in such a way that the axes x,y,z point in directions forward,up,right (respectively)
- The positive rotation for Yaw/Pitch movements is in the left/up directions (according to the 2nd rule of the right hand)
- The state variables of the gimbal are: 0->yaw_angle, 1->pitch_angle, 2->yaw_velocity, 3->pitch_velocity (it's important to mantain this order)
- The Pitch angle has to be in range [-30, +30] degrees (more or less), while Yaw angle can assume whatever value
**************************************************************/

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** model ***/
struct robot gimbal;

/*** control ***/
static float middle_yaw = 0;
static float middle_pitch = 0;
static float ref_yaw = 0;			// reference for Yaw
static float ref_pitch = 0;		// reference for Pitch
static float cmd_yaw = 0;			// yaw command received by remote controller or keyboard/mouse
static float cmd_pitch = 0;		// pitch command received by remote controller or keyboard/mouse
static float cmd_yaw_prev = 0;
static float cmd_pitch_prev = 0;
static float sat_u_yaw;			// saturation value for yaw control signal
static float sat_u_pitch;		// saturation value for pitch control signal
static float max_gimbal_pitch_tilt = 35*DEG_TO_RAD;

/*** gains to be tuned ***/
// gains (to be tuned manually) that should be such that the motors actually apply a torque equal to the desired one
static float gain_u_yaw = 1;
static float gain_u_pitch = 1;

/* PID control for yaw and pitch*/
static struct PID_ pid_yaw;
static struct PID_ pid_pitch;
#if IS_STD
// yaw
static float Kp_yaw	= 40;
static float Ki_yaw = 20;
static float Kd_yaw	= 2;
static float sat_ei_yaw = 0.2;
// pitch
static float Kp_pitch	= 40;//30
static float Ki_pitch	= 20;//35.6
static float Kd_pitch	= 2;//1.5
static float sat_ei_pitch = 0.5;

#elif IS_BR2
// yaw
static float Kp_yaw	= 40;
static float Ki_yaw	= 20;
static float Kd_yaw	= 2;
static float sat_ei_yaw	= 0.2;
// pitch
static float Kp_pitch	= 40;
static float Ki_pitch = 15;
static float Kd_pitch	= 2;
static float sat_ei_pitch	= 0.5;

#elif IS_SENTRY
// yaw
static float Kp_yaw	= 40;
static float Ki_yaw	= 20;
static float Kd_yaw	= 2;
static float sat_ei_yaw = 0.2;
// pitch
static float Kp_pitch	= 34.8;
static float Ki_pitch	= 35.6;
static float Kd_pitch	= 3.05;
static float sat_ei_pitch = 0.5;
#endif


/*** filters ***/
// derivative part of yaw's PID
static struct fir fir_pid_D_yaw;
static uint32_t fir_pid_D_yaw_len = 31;
static float fir_pid_D_yaw_coeff[] = {-0.000000000000000001, 0.000282970589546487, 0.001204877787865870, 0.002941370083944624, 0.005726606005070869, 0.009797711077828329, 0.015326522879102166, 0.022352296249991133, 0.030729748201279557, 0.040104674753698104, 0.049924704168249935, 0.059486405988859063, 0.068013147274924704, 0.074752090746301278, 0.079074740770195606, 0.080564266846284163, 0.079074740770195606, 0.074752090746301264, 0.068013147274924718, 0.059486405988859098, 0.049924704168249963, 0.040104674753698111, 0.030729748201279598, 0.022352296249991150, 0.015326522879102172, 0.009797711077828329, 0.005726606005070864, 0.002941370083944625, 0.001204877787865866, 0.000282970589546489, -0.000000000000000001};
// derivative part of pitch's PID
static struct fir fir_pid_D_pitch;
static uint32_t fir_pid_D_pitch_len = 31;
static float fir_pid_D_pitch_coeff[] = {-0.000000000000000001, 0.000282970589546487, 0.001204877787865870, 0.002941370083944624, 0.005726606005070869, 0.009797711077828329, 0.015326522879102166, 0.022352296249991133, 0.030729748201279557, 0.040104674753698104, 0.049924704168249935, 0.059486405988859063, 0.068013147274924704, 0.074752090746301278, 0.079074740770195606, 0.080564266846284163, 0.079074740770195606, 0.074752090746301264, 0.068013147274924718, 0.059486405988859098, 0.049924704168249963, 0.040104674753698111, 0.030729748201279598, 0.022352296249991150, 0.015326522879102172, 0.009797711077828329, 0.005726606005070864, 0.002941370083944625, 0.001204877787865866, 0.000282970589546489, -0.000000000000000001};
// proportional part of pitch's PID
static struct fir fir_pid_P_pitch;
static uint32_t fir_pid_P_pitch_len = 9;
static float fir_pid_P_pitch_coeff[] = {0.003043903340842960, -0.012523222453369552, -0.047697662795552619, 0.257129032473543961, 0.600095898869070332, 0.257129032473543961, -0.047697662795552626, -0.012523222453369559, 0.003043903340842961};


/*** CV-driven gimbal ***/
// gimbal auto-aim structure
struct cv_gimbal_auto_aim cv_gimbal;
// auto-aim shooting hyperparameters
float v0 = 13.5;
float min_shoot_distance = 1;				// [m]
float max_shoot_distance = 5;				// [m]
float max_enemy_lock_distance = 7;	// [m]
float max_shoot_freq = 5;						// [Hz]
float shoot_freq_enemy_warn = 0.5;	// [Hz]
float pitch_fit_coeff[CV_PITCH_FIT_COEFFS] = {-0.28, 0.124, -0.010847, -0.0124, 0.0, 0.61497144};
uint8_t yaw_filter_buf_size = 5;
uint8_t pitch_filter_buf_size = 10;
// sentry's patrol mode
#if IS_SENTRY
static float time_ms_begin_patrol;
static float yaw_begin_patrol;
static float yaw_time_ratio_patrol = 45*DEG_TO_RAD;			// 45 degrees each second
#endif



/*** flags ***/
static int8_t is_first_iter = 1;			// true if the current algorithm iteration is the first one
static int8_t need_to_set_middle_yaw = 1;
static int8_t need_to_set_init_mouse_gimbal_pos = 1;
static int8_t need_to_set_init_rc_gimbal_pos = 1;
#if IS_SENTRY
static int8_t is_sentry_patrol_first_iter = TRUE;
#endif

/*** system identification ***/
#if SYSID_GIMBAL
struct data_sample gimbal_yaw;
struct data_sample gimbal_yaw_dot;
struct data_sample gimbal_u_yaw;
//struct data_sample gimbal_ref_yaw;
#if !SYSID_RC
struct data_sample gimbal_u_sysid;
#if SYSID_WN
static float gain_u_sysid = 1;				// gain when motor's input is random gaussian distributed white noise
#elif SYSID_RAND_BIN
static float gain_u_sysid = 0.8;			// gain when motor's input is random binary distributed signal
#elif SYSID_RAND_UNIF
static float gain_u_sysid = 0.9;			// gain when motor's input is random uniformly distributed signal
#endif
#endif
#endif



/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void gimbal_control_loop() {
	
	if (is_first_iter) {
		
		// saturation of yaw/gimbal motors
		sat_u_yaw = MAX_GIMBAL_YAW_CONTROL_SIGNAL_AMPLITUDE;
		sat_u_pitch = MAX_GIMBAL_PITCH_CONTROL_SIGNAL_AMPLITUDE;
	
		/* system identification */
#if SYSID_GIMBAL
		data_sampling_init(&gimbal_yaw, 			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&gimbal_yaw_dot,		SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&gimbal_u_yaw, 		SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
#endif
		
#if SYSID_GIMBAL && !SYSID_RC
		// initialize motor's input for system identification
		data_sampling_init(&gimbal_u_sysid, SYSID_SENT_NUM_SAMPLES, SYSID_SEND_SAMPLES_PERIOD);
	
#if SYSID_WN
		// generate the white noise with random gaussian distribution
		wn_gauss_gen(gimbal_u_sysid.samples, SYSID_SENT_NUM_SAMPLES);
	
#elif SYSID_RAND_BIN
		// generate the motor's input with random binary distribution (in set {0,1})
		rand_bin_gen(gimbal_u_sysid.samples, SYSID_SENT_NUM_SAMPLES);
	
		// convert all binary values from {0,1} to {-1,1}
		for (int i = 0; i < SYSID_SENT_NUM_SAMPLES; i++) {
			gimbal_u_sysid.samples[i] -= 0.5;
			gimbal_u_sysid.samples[i] *= 2;
		}
	
#elif SYSID_RAND_UNIF
		// generate the motor's input with random uniform distribution (in range [0,1])
		rand_unif_gen(gimbal_u_sysid.samples, SYSID_SENT_NUM_SAMPLES);
		
		// convert all binary values from range [0,1] to range [-1,1]
		for (int i = 0; i < SYSID_SENT_NUM_SAMPLES; i++) {
			gimbal_u_sysid.samples[i] -= 0.5;
			gimbal_u_sysid.samples[i] *= 2;
		}
	
#endif

		// normalize motor's input such that it stimulates the entire yaw dynamics
		float max_abs_u = max_abs(gimbal_u_sysid.samples, SYSID_SENT_NUM_SAMPLES);
		
		for (unsigned int i = 0; i < SYSID_SENT_NUM_SAMPLES; i++) {		
			gimbal_u_sysid.samples[i] *= (sat_u_yaw / max_abs_u) * gain_u_sysid;
		}

#endif
		
		/* controllers initialization */
		PID_init_(&pid_yaw, 	Kp_yaw, 	Ki_yaw, 	Kd_yaw, 		dt_gimbal);
		PID_init_(&pid_pitch, Kp_pitch, Ki_pitch, Kd_pitch, 	dt_gimbal);
		
		/* filters initialization */
		fir_init(&fir_pid_D_yaw, fir_pid_D_yaw_len, fir_pid_D_yaw_coeff, FALSE);
		fir_init(&fir_pid_D_pitch, fir_pid_D_pitch_len, fir_pid_D_pitch_coeff, FALSE);
		fir_init(&fir_pid_P_pitch, fir_pid_P_pitch_len, fir_pid_P_pitch_coeff, FALSE);
		
		// initialize CV for gimbal auto-aim
		cv_gimbal_auto_aim_init(&cv_gimbal, v0, yaw_filter_buf_size, pitch_filter_buf_size, pitch_fit_coeff,
														min_shoot_distance, max_shoot_distance, max_shoot_freq, max_enemy_lock_distance, shoot_freq_enemy_warn);
	}
	
	/* take relative Yaw position of chassis w.r.t. gimbal */
	br_chassis.x[6] = ((float) (((int) ((get_yaw_gimbal_motor_measures()->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET) * 180/pi)) % 360) * pi/180);
	/* NOTE: the "... % 360" is used to avoid doing more than 360 deg to align chassis with gimbal */
	
	// [input] take board measurements
#if IS_STD || IS_SENTRY
	gimbal.x[0] = (3.1415/180)*ins_correct_angle[2];			// angular position of motor 0x205 (Yaw of gimbal)
	gimbal.x[1] = (TAKE_PITCH_FROM_MOTOR_ENCODER) ? get_pitch_gimbal_motor_measures()->contig_ang_pos_rad : (3.1415/180)*ins_correct_angle[0]; 	// angular position of motor 0x206 (Pitch of gimbal)
	gimbal.x[2] = gz;	// angular velocity of motor 0x205
	gimbal.x[3] = (TAKE_PITCH_FROM_MOTOR_ENCODER) ? get_pitch_gimbal_motor_measures()->ang_vel_radsec : gx;	// angular velocity of motor 0x206
	
#elif IS_BR2
	/* yaw/pitch positions from encoder and microcontroller */
	float yaw_encoder 	= get_yaw_gimbal_motor_measures()->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET;
	float yaw_uC 				= (pi/180)*ins_correct_angle[2];
	float pitch_encoder = get_pitch_gimbal_motor_measures()->contig_ang_pos_rad - CENTER_ENCODER_ANGLE_PITCH;
	float pitch_uC 			= (pi/180)*ins_correct_angle[0] * cos(br_chassis.x[6]) * cos(br_chassis.x[6])
												+
												(pi/180)*ins_correct_angle[1] * sin(br_chassis.x[6]) * sin(br_chassis.x[6]);
	
	/* yaw/pitch velocities from encoder and microcontroller */
	float yaw_dot_encoder 	= get_yaw_gimbal_motor_measures()->ang_vel_radsec;
	float yaw_dot_uC 				= gz;
	float pitch_dot_encoder	= get_pitch_gimbal_motor_measures()->ang_vel_radsec;
	float pitch_dot_uC 			= gx * cos(br_chassis.x[6]) * cos(br_chassis.x[6])
														+
														gy * sin(br_chassis.x[6]) * sin(br_chassis.x[6]);

	// if the chassis' backward (NOT forward) part
	if (ref_yaw_chassis_dir(0, br_chassis.x[6]) == -1) {
		
		pitch_uC 			*= -1;
		pitch_dot_uC	*= -1;
	}
	
	/* yaw/pitch position/velocity references */
	gimbal.x[0] = yaw_encoder + yaw_uC;
	gimbal.x[1] = pitch_encoder + pitch_uC;
	gimbal.x[2] = yaw_dot_encoder + yaw_dot_uC;
	gimbal.x[3] = pitch_dot_encoder + pitch_dot_uC;
	
#endif
	
	/* reference signals */
#if IS_STD || IS_BR2
		
	if (switch_is_up(rc_right_switch)) {		// Call-of-Duty-like control with the joystick

		// [input] take references for yaw and pitch from the joystick
		cmd_yaw 	= (((float)(- get_remote_control_point()->rc.ch[0])) / MAX_RC_TILT) * (15*DEG_TO_RAD);
		cmd_pitch = (((float)(+ get_remote_control_point()->rc.ch[1])) / MAX_RC_TILT) * (30*DEG_TO_RAD);
		
		// set pitch lower bound
		if (cmd_pitch < -17*DEG_TO_RAD)
			cmd_pitch = -17*DEG_TO_RAD;


		// next time that we switch to keyboard/mouse commands, we'll set a new starting position for the mouse
		if (!need_to_set_init_mouse_gimbal_pos)
			need_to_set_init_mouse_gimbal_pos = 1;

		/* decide if we need to set a new "middle yaw" position */
		if (!need_to_set_middle_yaw && cmd_yaw != 0) {
			need_to_set_middle_yaw = 1;
		}
		if (need_to_set_middle_yaw || need_to_set_init_rc_gimbal_pos) {
			middle_yaw = gimbal.x[0];
			need_to_set_middle_yaw = 0;
			need_to_set_init_rc_gimbal_pos = 0;
		}

		// set the reference signals
		ref_yaw = middle_yaw + cmd_yaw;
		ref_pitch = middle_pitch + cmd_pitch;

	}
	else if (switch_is_mid(rc_right_switch)) {		// Minecraft-like control with the mouse

		// next time that we switch to keyboard/mouse commands, we'll set a new starting position of the gimbal
		if (!need_to_set_init_rc_gimbal_pos)
			need_to_set_init_rc_gimbal_pos = 1;

		if (need_to_set_init_mouse_gimbal_pos) {
			
			cmd_yaw 	= gimbal.x[0];
			cmd_pitch = gimbal.x[1];
			mouse_speed_x_prev = 0;
			mouse_speed_y_prev = 0;
			need_to_set_init_mouse_gimbal_pos = 0;
		}
		
		
		if (IS_PILOT_AUTO_AIM_ENABLED && is_mouse_right_key_pressed()) {
			
			/* get CV track references for yaw and pitch */
			cv_enemy_track(&cmd_yaw, &cmd_pitch, NULL, &cv_gimbal, gimbal.x[0], gimbal.x[1]);				// TODO substitute cmd_yaw and cmd_pitch with something else (probably it doesn't work like this)
		}
		else {
		
			// this function uses 'mouse_meters_yaw_rad_ratio' and 'mouse_meters_pitch_rad_ratio' variables to convert from mouse's meters to gimbal's radiants
			update_mouse_position_gimbal(dt_gimbal, &cmd_yaw, &cmd_pitch);
		}
		
		// saturation due to mechanical limit in pitch movements
		saturate(&cmd_pitch, max_gimbal_pitch_tilt);

		// set the reference signals
		ref_yaw = cmd_yaw;
		ref_pitch = cmd_pitch;

	}
#elif IS_SENTRY
	
	if (switch_is_up(rc_right_switch)) {		// Call-of-Duty-like control with the joystick

		// [input] take references for yaw and pitch from the joystick
		cmd_yaw 	= (((float)(- get_remote_control_point()->rc.ch[0])) / MAX_RC_TILT) * (15*DEG_TO_RAD);
		cmd_pitch = (((float)(+ get_remote_control_point()->rc.ch[1])) / MAX_RC_TILT) * (30*DEG_TO_RAD);
		
		// set pitch lower bound
		if (cmd_pitch < -17*DEG_TO_RAD)
			cmd_pitch = -17*DEG_TO_RAD;

		
		/* decide if we need to set a new "middle yaw" position */
		if (!need_to_set_middle_yaw && cmd_yaw != 0) {
			need_to_set_middle_yaw = 1;
		}
		if (need_to_set_middle_yaw || need_to_set_init_rc_gimbal_pos) {
			middle_yaw = gimbal.x[0];
			need_to_set_middle_yaw = 0;
			need_to_set_init_rc_gimbal_pos = 0;
		}

		// set the reference signals
		ref_yaw = middle_yaw + cmd_yaw;
		ref_pitch = middle_pitch + cmd_pitch;

	}
	else if (switch_is_mid(rc_right_switch)) {		// CV-driver auto-aim gimbal control
		
		// next time that we switch to keyboard/mouse commands, we'll set a new starting position of the gimbal
		if (!need_to_set_init_rc_gimbal_pos)
			need_to_set_init_rc_gimbal_pos = 1;
		
		if (sentry_state == IDLE) {			// NOTE: we should never enter in this "if"
			
			// stay idle
			ref_yaw = gimbal.x[0];
			ref_pitch = gimbal.x[1];
		}
		if (sentry_state == MOVE_TO_SPOT) {
			
			// mantain gimbal's YAW and PITCH to the STARTING and HORIZONTAL position, respectively
			ref_yaw = nearest_contig_ref_angle(0.0, gimbal.x[0]);
			ref_pitch = 0;
		}
		if (sentry_state == PATROL) {
			
			if (is_sentry_patrol_first_iter || !switch_is_mid(rc_right_switch_prev)) {
				
				time_ms_begin_patrol = HAL_GetTick();
				yaw_begin_patrol = gimbal.x[0];
				is_sentry_patrol_first_iter = FALSE;				// NOTE: this flag gets set again to 1 at the end of the this code (if some criteria are fulfilled)
			}
			
			ref_yaw = yaw_begin_patrol + yaw_time_ratio_patrol * ((HAL_GetTick() - time_ms_begin_patrol)*1e-3);
			
			// mantain the gimbal's pitch horizontal
			ref_pitch = 0;
		}
		else {
			
			/* here sentry_state is either ENEMY_LOCK, ENEMY_WARN and ENEMY_ENGAGE */
			
			if ((sentry_state_prev != ENEMY_LOCK && sentry_state_prev != ENEMY_WARN && sentry_state_prev != ENEMY_ENGAGE) ||
					(!switch_is_mid(rc_right_switch_prev))) {
					
				// reset yaw reference
				ref_yaw = gimbal.x[0];
				
				/* reset yaw/pitch filters */
				for (uint8_t i = 0; i < cv_gimbal.fir_yaw_cv.len; i++)
					cv_gimbal.fir_yaw_cv.buf[i] = gimbal.x[0];
			
				for (uint8_t i = 0; i < cv_gimbal.fir_pitch_cv.len; i++)
					cv_gimbal.fir_pitch_cv.buf[i] = gimbal.x[1];
			}
			
			/* get CV track references for yaw, pitch and shooting frequency */
			cv_enemy_track(&ref_yaw, &ref_pitch, &cv_shoot_freq, &cv_gimbal, gimbal.x[0], gimbal.x[1]);


			/* 1st OLD APPROACH */

//			// get enemy's x,y,z coordinates from stereo camera
//			x_cam_left = - get_AI_ref1();
//			y_cam_left = + get_AI_ref0();
//			z_cam_left = + get_AI_ref2();

//			// add the offsets due to the ZED's physical structure
//			x_cam = x_cam_left - distance_left_cam_to_center_zed2;
//			y_cam = y_cam_left;
//			z_cam = z_cam_left + distance_center_zed2_to_center_shooting;

//			/* compute yaw/pitch references with respect to the current yaw/pitch angle */
//			if (x_cam_left == 0 && y_cam_left == 0 && z_cam_left == 0) {
//				
//				// ZED detection didn't find any enemy
//				cmd_pitch = 0;
//				cmd_yaw = 0;
//			}
//			else {
//				
//				// ZED detected an enemy, so we have to aim to it
//				cmd_yaw 	= atan2(- x_cam / sqrt(x_cam*x_cam + y_cam*y_cam), + y_cam / sqrt(x_cam*x_cam + y_cam*y_cam));
//				
//				
//				float d_armor = y_cam*cos(gimbal.x[1]) - z_cam*sin(gimbal.x[1]);                  // distance from enemy's armor (on ground, i.e., along Y axis)
//				float h_armor = h_shooter + y_cam*sin(gimbal.x[1]) + z_cam*cos(gimbal.x[1]);      // altitude of enemy's armor (from ground)

//				
//				cmd_pitch = pitch_fit_coeff[0] +
//										d_armor * pitch_fit_coeff[1] +
//										h_armor * pitch_fit_coeff[2] +
//										d_armor * d_armor * pitch_fit_coeff[3] +
//										d_armor * h_armor * pitch_fit_coeff[4] +
//										h_armor * h_armor * pitch_fit_coeff[5];
//				
//				// TODO delete
//				//cmd_pitch = atan2(+ z_cam / sqrt(z_cam*z_cam + y_cam*y_cam), + y_cam / sqrt(z_cam*z_cam + y_cam*y_cam));
//			}
//			
//				
//			/* set the reference signals */
//			if (x_cam_left != x_cam_left_prev || y_cam_left != y_cam_left_prev || z_cam_left != z_cam_left_prev) {
//				
//				// a new info just came from the stereocamera, so update the reference signal
//				ref_yaw 	= gimbal.x[0] + cmd_yaw;
//				ref_pitch = gimbal.x[1] + cmd_pitch;
//				
//				// update the previous x,y,z values obtained from the stereocamera
//				x_cam_left_prev = x_cam_left;
//				y_cam_left_prev = y_cam_left;
//				z_cam_left_prev = z_cam_left;
//			}
//			
//			/* for both yaw and pitch, check if the value received from ZED detection is NaN; if yes, then substitute it with the last not-NaN receive value */
//			if (isnan(ref_yaw))
//				ref_yaw = fir_yaw_cv.buf[fir_yaw_cv.buf_offset];
//			if (isnan(ref_pitch))
//				ref_pitch = fir_pitch_cv.buf[fir_pitch_cv.buf_offset];
//			
//			/* filter the yaw/pitch references (obtained by the ZED with a low-pass filter */
//			ref_yaw = fir_filter(&fir_yaw_cv, ref_yaw);
//			ref_pitch = fir_filter(&fir_pitch_cv, ref_pitch);
		
		
		
			/* 2nd OLD APPROACH */
		
//			// get enemy's x,y,z coordinates from stereo camera
//			x_cam_left = - get_AI_ref1();
//			y_cam_left = + get_AI_ref0();
//			z_cam_left = + get_AI_ref2();
//			
//			
//			if (x_cam_left == 0 || y_cam_left == 0 || z_cam_left == 0 || isnan(x_cam_left) || isnan(y_cam_left) || isnan(z_cam_left)) {
//				
//				// stereo camera is not finding any enemy
//				
//				/* yaw remains where it is */
//				ref_yaw 	= gimbal.x[0];
//				
//				/* pitch initially remains where it is, but after some time it has to return to "patrol" mode */
//				ref_pitch = gimbal.x[1];
//				// ... (enable patrol mode after some time)
//			}
//			else {
//				
//				// stereo camera found an enemy
//				
//				/* add the offsets due to the ZED's physical structure */
//				x_cam = x_cam_left - distance_left_cam_to_center_zed2;
//				y_cam = y_cam_left;
//				z_cam = z_cam_left  + distance_center_zed2_to_center_shooting;
//				
//				/* get CV track references for yaw, pitch and shooting frequency */
//				cv_enemy_track(&ref_yaw, &ref_pitch, &cv_shoot_freq, &cv_gimbal, x_cam, y_cam, z_cam, gimbal.x[1]);
//			
//				ref_yaw 	+= gimbal.x[0];
//				ref_pitch += 0;
//				
//				// update the previous x,y,z values obtained from the stereocamera
//				x_cam_left_prev = x_cam_left;
//				y_cam_left_prev = y_cam_left;
//				z_cam_left_prev = z_cam_left;
//			}
		}
	}

#endif
	
	
	/* pitch should point downwards when doing system identification on yaw */
#if SYSID_GIMBAL
	ref_pitch = -26*DEG_TO_RAD;
#endif
	
	
	/* set yaw/pitch position references */
	if (!isnan(ref_yaw) && !isinf(ref_yaw))
		gimbal.ref[0] = ref_yaw;
	
	if (!isnan(ref_pitch) && !isinf(ref_pitch))
		gimbal.ref[1] = ref_pitch;
	
	/* set yaw/pitch velocity references */
	gimbal.ref[2] = 0;
	gimbal.ref[3] = 0;
	
	
	/* saturate pitch reference */
	saturate_in_range(&gimbal.ref[1], -23*DEG_TO_RAD, 25*DEG_TO_RAD);
	
	
	/* compute error w.r.t. references */
	for (int i = 0; i < 4; i++)
		ref_error_state(&gimbal, i, dt_gimbal);
	
	// saturate gimbal's yaw and pitch integral error
	saturate(&gimbal.integr_error[0], sat_ei_yaw);
	saturate(&gimbal.integr_error[1], sat_ei_pitch);
	
	/* compute control signal */
	gimbal.u[0] = PID_control(&pid_yaw,		gimbal.error[0], 	gimbal.integr_error[0], 	gimbal.error[2],
																				UNDEF,					 	UNDEF,										UNDEF,
																				NULL,						 	NULL,										&fir_pid_D_yaw);
	gimbal.u[1] = PID_control(&pid_pitch, gimbal.error[1], 	gimbal.integr_error[1], 	gimbal.error[3],
																				UNDEF,					 	UNDEF,										UNDEF,
																				&fir_pid_P_pitch,	NULL,										&fir_pid_D_pitch);
	
	
	// from torque to control signal (to be sent to the GM6020 motor)
	gimbal.u[0] *= gain_u_yaw;
	gimbal.u[1] *= gain_u_pitch;
	
#if SYSID_GIMBAL && !SYSID_RC
	gimbal.u[0] = send_data_sample(&gimbal_u_sysid);		// send signal to yaw motor (for system identification)
#endif
	
	// discrete-torque ratio of GM6020 motor
	gimbal.u[0] *= GM6020_DISCRETE_TORQUE_RATIO;
	gimbal.u[1] *= GM6020_DISCRETE_TORQUE_RATIO;
	
	// saturation of the control signals (in order to avoid possible damages)
	saturate(&gimbal.u[0], sat_u_yaw);
	saturate(&gimbal.u[1], sat_u_pitch);
	
	// [output] send the control signals
	if (IS_GIMBAL_ENABLED)
		CAN_cmd_gimbal((int16_t) gimbal.u[0], (int16_t) gimbal.u[1]);
	
	// set previous references, state estimations and errors for next iteration
	for (int i = 0; i < 4; i++) {
		
		gimbal.ref_prev[i] 	 = gimbal.ref[i];
		gimbal.x_prev[i] 		 = gimbal.x[i];
		gimbal.error_prev[i] = gimbal.error[i];
	}
	
	// set previous commands from joystick/mouse for next iteration
	cmd_yaw_prev = cmd_yaw;
	cmd_pitch_prev = cmd_pitch;
	
	// reset the flag for first iteration of sentry's patrol mode
#if IS_SENTRY
	if (sentry_state != PATROL)
		is_sentry_patrol_first_iter = TRUE;
#endif
	
	
	/* acquire data for system identification */
#if SYSID_GIMBAL
	acquire_data_sample(&gimbal_yaw, 			gimbal.x[0] * RAD_TO_DEG);
	acquire_data_sample(&gimbal_yaw_dot, 	gimbal.x[2] * RAD_TO_DEG);
	acquire_data_sample(&gimbal_u_yaw, 		gimbal.u[0] * GM6020_DAC);
//	acquire_data_sample(&gimbal_ref_yaw, 	gimbal.ref[0] * RAD_TO_DEG);
#endif
	
	// first algorithm iteration terminates here
	is_first_iter = 0;
}



