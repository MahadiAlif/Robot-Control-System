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
#include "sentry_chassis_control.h"
#include "AI_receive.h"
#include "robot_config.h"


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
gimbal_model_t standard_gimbal;		// model of the standard robot's gimbal

/*** control ***/
float T_gimbal = 0.001; 		// inverse of the frequency of gimbal controller (1000 Hz)
float gimbal_yaw_LQR_K[2];		// LQR gain of the standard robot's gimbal yaw
float gimbal_pitch_LQR_K[2];		// LQR gain of the standard robot's gimbal pitch
float standard_gimbal_I_gain[2];	// integral gain for error w.r.t. gimbal's position
float current_middle_yaw = 0;
float current_middle_pitch = 0;
float ref_gimbal_yaw = 0;			// reference for Yaw
float ref_gimbal_pitch = 0;		// reference for Pitch
float cmdYawGimbal = 0;			// Yaw command sent by the joystick/mouse
float cmdPitchGimbal = 0;		// Pitch command sent by the joystick/mouse
float cmdYawGimbal_prev = 0;
float cmdPitchGimbal_prev = 0;
float saturation_control_signal_yaw_gimbal;			// Yaw control signal saturation value
float saturation_control_signal_pitch_gimbal;		// Pitch control signal saturation value
float max_gimbal_pitch_inclination = 35*pi/180;



/*** gains to be tuned ***/
// gains (to be tuned manually) that should be such that the motors actually apply a torque equal to the desired one
float gain_torque_to_control_signal_yaw = 1;		// 1.8
float gain_torque_to_control_signal_pitch = 1;	// 2
// LQR Yaw and Pitch gains
float gain_LQR_control_signal_gimbal_yaw = (IS_STD) ? 1600 : ((IS_SENTRY) ? 1600 : ((IS_BR1) ? 2000 : /*2.0*/2000));
float gain_LQR_control_signal_gimbal_pitch = (IS_STD) ? 1600 : ((IS_SENTRY) ? 1600 : ((IS_BR1) ? 2500 : /*2.5*/2500));
// gains to mantain a certain orientation of the gimbal
float gain_mantain_position_yaw = 10;
float gain_mantain_position_pitch = 10;
// gain to compensate the fact that the center of mass of the gimbal is inclined backward
float gain_pitch_goes_down = (IS_STD) ? /*1.9*/1 : ((IS_SENTRY) ? 1.9 : ((IS_BR1) ? 1.5 : 1/*1.5*/));	// 4.5 quando è full proiettili


/* PID control for yaw and pitch*/
static struct PID_ pid_yaw;
static struct PID_ pid_pitch;
#if IS_STD

static float Kp_pitch = 34.8;
static float Ki_pitch = 35.6;
static float Kd_pitch = 3.05;
static float N_pitch = 470;
float sat_ei_pitch = 0.3;

#elif IS_BR1 || IS_BR2 || IS_LBR1 || IS_LBR2
// yaw
static float Kp_yaw 		= 40;
static float Ki_yaw 		= 20;
static float Kd_yaw 		= 2;			// this should be higher, but first we need to implement a better low-pass filter
static float N_yaw 			= 30;
float sat_ei_yaw 				= 0.2;
// pitch
static float Kp_pitch 	= 30;		// 34.8
static float Ki_pitch 	= 15;			// 35.6
static float Kd_pitch 	= 2;
float sat_ei_pitch 			= 0.3;
#endif



/*** exponential gain values ***/
// maximum values (for Yaw and Pitch) of the gains that, the more Yaw and Pitch are near to their reference, the more is large (minimum gain is fixed to 1)
float max_exp_yaw_error_gain = 1.5;
float max_exp_pitch_error_gain = (IS_STD) ? 2 : ((IS_SENTRY) ? 2 : ((IS_BR1) ? 1.1 : 1.1));
// Yaw and Pitch angles at which the exponential gain is = 1
float max_exp_gain_yaw_range = pi/2;
float max_exp_gain_pitch_range = 20*pi/180;
// points between 0 and "max_exp_gain_yaw_range"/"max_exp_gain_pitch_range" that determines the exponent of the exponential gain
float middle_angle_exp_yaw = pi/4;
float middle_angle_exp_pitch = 10*pi/180;
// values of the exponential gain corresponding to "middle_angle_exp_yaw" and "middle_angle_exp_pitch"
float middle_value_exp_yaw = 1.23;
float middle_value_exp_pitch = (IS_STD) ? 1.2 : ((IS_SENTRY) ? 1.2 : ((IS_BR1) ? 1.02 : 1.02));

/*** gimbal mantain position values ***/
float yaw_angle_where_mantain_position_starts;		// x1 for yaw
float pitch_angle_where_mantain_position_starts;	// x1 for pitch
float length_yaw_gain_increase = 15.0*pi/180;			// x4-x1 for yaw
float length_pitch_gain_increase = 5.0*pi/180;		// x4-x1 for pitch
float max_gain_mantain_yaw = 5;										// ymax for yaw
float max_gain_mantain_pitch = 5;									// ymax for pitch
float epsilon_yaw = 0.1;													// in (x1 + (x1+x4)/4) the gain is 1+epsilon
float epsilon_pitch = 0.5;												// in (x1 + (x1+x4)/4) the gain is 1+epsilon

/*** gimbal yaw and pitch integral values ***/
float gain_integral_yaw_error = (IS_STD) ? 70 : ((IS_SENTRY) ? 0 : 0);
float gain_integral_pitch_error = (IS_STD) ? 0 : ((IS_SENTRY) ? 0 : 0); // 30, oppure 0 va bene lo stesso (provato quando era full proiettili)
float range_where_trigger_yaw_integral = 5*pi/180;
float range_where_trigger_pitch_integral = 17*pi/180;

/*** gimbal mouse movement values ***/
float mouse_meters_yaw_rad_ratio = 0.016375;		// if mouse travels 0.01 meters, then yaw has to travel 35*pi/180 rad
float mouse_meters_pitch_rad_ratio = 0.01;		// if mouse travels 0.005 meters, then yaw has to travel 30*pi/180 rad
uint32_t clock_ms_mouse_gimbal;
uint32_t clock_ms_mouse_gimbal_prev = 0;

/*** sentry values to scan environment and search enemies ***/
float sentry_yaw_freq_scan_environment = 0.3;
float sentry_pitch_freq_scan_environment = 0.7;
float sentry_yaw_amplitude_scan_environment = pi/4;
float sentry_pitch_amplitude_scan_environment = 20*pi/180;

/*** sentry references taken from AI algorithm ***/
// x,y,z w.r.t. camera sx of ZED stereocamera
fp32 zed_x_camera_sx = 0;
fp32 zed_y_camera_sx = 0;
fp32 zed_z_camera_sx = 0;
// x,y,z w.r.t. camera sx of ZED stereocamera in previous time instants
fp32 zed_x_camera_sx_prev = 0;
fp32 zed_y_camera_sx_prev = 0;
fp32 zed_z_camera_sx_prev = 0;
// x,y,z w.r.t. center of ZED stereocamera
fp32 zed_x;
fp32 zed_y;
fp32 zed_z;

fp32 d;		// distance from center of ZED stereocamera to detected object
fp32 camera_sx_to_center_zed = 0.06;		// distance between camera sx and center of ZED stereocamera
fp32 center_zed_to_center_shooting;


/*** flags ***/
int need_to_initialize_gimbal = 1;
int need_to_mantain_gimbal_yaw_position = 1;
int need_to_mantain_gimbal_pitch_position = 1;
int need_to_mantain_gimbal_yaw_position_prev = 0;
int need_to_mantain_gimbal_pitch_position_prev = 0;
int need_to_set_middle_yaw = 1;
int need_to_set_middle_pitch = 1;
int need_to_set_gain_pitch = 1;
int need_to_set_initial_mouse_gimbal_position = 1;
int need_to_set_initial_joystick_gimbal_position = 1;
int pitch_is_going_down = 0;	// check if pitch of gimbal if going either down or up (useful to compute the balance angle of BR at runtime)


/* system identification */
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



static struct fir fir_pid_D_yaw;
static uint32_t fir_pid_D_yaw_len = 31;
static float fir_pid_D_yaw_coeff[] = {-0.000000000000000001, 0.000282970589546487, 0.001204877787865870, 0.002941370083944624, 0.005726606005070869, 0.009797711077828329, 0.015326522879102166, 0.022352296249991133, 0.030729748201279557, 0.040104674753698104, 0.049924704168249935, 0.059486405988859063, 0.068013147274924704, 0.074752090746301278, 0.079074740770195606, 0.080564266846284163, 0.079074740770195606, 0.074752090746301264, 0.068013147274924718, 0.059486405988859098, 0.049924704168249963, 0.040104674753698111, 0.030729748201279598, 0.022352296249991150, 0.015326522879102172, 0.009797711077828329, 0.005726606005070864, 0.002941370083944625, 0.001204877787865866, 0.000282970589546489, -0.000000000000000001};

static struct fir fir_pid_D_pitch;
static uint32_t fir_pid_D_pitch_len = 31;
static float fir_pid_D_pitch_coeff[] = {-0.000000000000000001, 0.000282970589546487, 0.001204877787865870, 0.002941370083944624, 0.005726606005070869, 0.009797711077828329, 0.015326522879102166, 0.022352296249991133, 0.030729748201279557, 0.040104674753698104, 0.049924704168249935, 0.059486405988859063, 0.068013147274924704, 0.074752090746301278, 0.079074740770195606, 0.080564266846284163, 0.079074740770195606, 0.074752090746301264, 0.068013147274924718, 0.059486405988859098, 0.049924704168249963, 0.040104674753698111, 0.030729748201279598, 0.022352296249991150, 0.015326522879102172, 0.009797711077828329, 0.005726606005070864, 0.002941370083944625, 0.001204877787865866, 0.000282970589546489, -0.000000000000000001};

static struct fir fir_pid_P_pitch;
static uint32_t fir_pid_P_pitch_len = 9;
static float fir_pid_P_pitch_coeff[] = {0.003043903340842960, -0.012523222453369552, -0.047697662795552619, 0.257129032473543961, 0.600095898869070332, 0.257129032473543961, -0.047697662795552626, -0.012523222453369559, 0.003043903340842961};


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void gimbal_control_loop(uint8_t robot_control_mode) {
	
	if (need_to_initialize_gimbal) {
		
		gimbal_control_init();
		
		fir_init(&fir_pid_D_yaw, fir_pid_D_yaw_len, fir_pid_D_yaw_coeff);
		fir_init(&fir_pid_D_pitch, fir_pid_D_pitch_len, fir_pid_D_pitch_coeff);
		fir_init(&fir_pid_P_pitch, fir_pid_P_pitch_len, fir_pid_P_pitch_coeff);
		
		PID_init_(&pid_yaw, 	Kp_yaw, 	Ki_yaw, 	Kd_yaw, 		0.001);
		PID_init_(&pid_pitch, Kp_pitch, Ki_pitch, Kd_pitch, 	0.001);
		
		need_to_initialize_gimbal = 0;
		
#if SYSID_GIMBAL
		data_sampling_init(&gimbal_yaw, 			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&gimbal_yaw_dot,		SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&gimbal_u_yaw, 		SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
//		data_sampling_init(&gimbal_ref_yaw, 	SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
#endif
	}
	
	// check commands from keyboard to gimbald
	if (IS_STD || IS_BR1 || IS_BR2)
		check_keyboard_commands_gimbal(robot_control_mode);
	
	/* take relative Yaw position of chassis w.r.t. gimbal */
	br_chassis.x[6] = ((float) (((int) ((get_yaw_gimbal_motor_measures()->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET) * 180/pi)) % 360) * pi/180);
	/* NOTE: the "... % 360" is used to avoid doing more than 360 deg to align chassis with gimbal */
	
	// [input] take board measurements
	/*NOTE: for the gimbal's pitch we take the measurements from the GM6020 encoder (not from the board) */
	if (IS_STD) {
		standard_gimbal.y[0] = (3.1415/180)*ins_correct_angle[2];			// angular position of motor 0x205 (Yaw of gimbal)
		standard_gimbal.y[1] = (TAKE_PITCH_FROM_MOTOR_ENCODER) ? get_pitch_gimbal_motor_measures()->contig_ang_pos_rad : (3.1415/180)*ins_correct_angle[0]; 	// angular position of motor 0x206 (Pitch of gimbal)
		standard_gimbal.y[2] = gz;	// angular velocity of motor 0x205
		standard_gimbal.y[3] = (TAKE_PITCH_FROM_MOTOR_ENCODER) ? get_pitch_gimbal_motor_measures()->ang_vel_radsec : gx;	// angular velocity of motor 0x206
	}
	else if (IS_BR1 || IS_BR2) {
		standard_gimbal.y[0] = (get_yaw_gimbal_motor_measures()->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET) + (3.1415/180)*ins_correct_angle[2];
		standard_gimbal.y[1] = (get_pitch_gimbal_motor_measures()->contig_ang_pos_rad - CENTER_ENCODER_ANGLE_PITCH)
														+
														(3.1415/180)*ins_correct_angle[0]
														* cos(br_chassis.x[6]) * cos(br_chassis.x[6])
														+
														(3.1415/180)*ins_correct_angle[1]
														* sin(br_chassis.x[6]) * sin(br_chassis.x[6]);
		standard_gimbal.y[2] = get_yaw_gimbal_motor_measures()->ang_vel_radsec + gz;
		standard_gimbal.y[3] = get_pitch_gimbal_motor_measures()->ang_vel_radsec
														+
														gx
														* cos(br_chassis.x[6]) * cos(br_chassis.x[6])
														+
														gy
														* sin(br_chassis.x[6]) * sin(br_chassis.x[6]);
	}
	
	
	// compute the state estimations: standard_gimbal.state_estim[i] will be updated by LPF
	for (int i = 0; i < 4; i++)
	{
		standard_gimbal.state_estim[i] = standard_gimbal.y[i];
	}
	
	
	// reference signals
	if (IS_STD || IS_BR1 || IS_BR2) {
		
		if (robot_control_mode == REMOTE_CONTROLLER_MODE || robot_control_mode == FIXED_GIMBAL_PC_MODE) {		// Call-of-Duty-like control with the joystick
			
			// [input] take references for Yaw and Pitch from the joystick
			if (!NEED_TO_TEST_SENTRY_FUNCTIONALITIES_ON_BALANCING)
			{
				cmdYawGimbal = (((float)(- get_remote_control_point()->rc.ch[0]))/MAX_RC_TILT)*(15*pi/180);
				cmdPitchGimbal = (((float)(get_remote_control_point()->rc.ch[1]))/MAX_RC_TILT)*(30*pi/180);
				if (cmdPitchGimbal < -17*pi/180)
					cmdPitchGimbal = -17*pi/180;
			}
			else
			{
				zed_x_camera_sx = - get_AI_ref0();
				zed_y_camera_sx = get_AI_ref1();
				zed_z_camera_sx = - get_AI_ref2();
				zed_x = zed_x_camera_sx + camera_sx_to_center_zed;
				zed_y = zed_y_camera_sx + center_zed_to_center_shooting;
				zed_z = zed_z_camera_sx;
				d = sqrt(zed_x*zed_x + zed_y*zed_y + zed_z*zed_z);
				if (zed_x_camera_sx == 0 && zed_y_camera_sx == 0 && zed_z_camera_sx == 0) {
					cmdPitchGimbal = 0;
					cmdYawGimbal = 0;
				}
				else {
					cmdPitchGimbal = atan2(zed_y / sqrt(zed_y*zed_y + zed_z*zed_z), zed_z / sqrt(zed_y*zed_y + zed_z*zed_z));
					cmdYawGimbal = atan2(zed_x / sqrt(zed_x*zed_x + zed_z*zed_z), zed_z / sqrt(zed_x*zed_x + zed_z*zed_z));
				}
				
				
			}
			
			// next time that we switch to keyboard/mouse commands, we'll set a new starting position for the mouse
			if (!need_to_set_initial_mouse_gimbal_position)
				need_to_set_initial_mouse_gimbal_position = 1;
		
			// decide if we need to set a new "middle yaw" and "middle pitch" position
			if (!need_to_set_middle_yaw && cmdYawGimbal != 0) {
				need_to_set_middle_yaw = 1;
			}
			if (!need_to_set_middle_pitch && cmdPitchGimbal != 0) {
				need_to_set_middle_pitch = 1;
			}
			if (need_to_set_middle_yaw || need_to_set_initial_joystick_gimbal_position) {
				current_middle_yaw = standard_gimbal.state_estim[0];
				need_to_set_middle_yaw = 0;
				need_to_set_initial_joystick_gimbal_position = 0;
			}
			if (need_to_set_middle_pitch || need_to_set_initial_joystick_gimbal_position) {
				current_middle_pitch = 0; //standard_gimbal.state_estim[1] - ((IS_BR1 || IS_BR2) ? CENTER_ENCODER_ANGLE_PITCH : 0);
				need_to_set_middle_pitch = 0;
				need_to_set_initial_joystick_gimbal_position = 0;
			}
				
			// set the reference signals
			if (!NEED_TO_TEST_SENTRY_FUNCTIONALITIES_ON_BALANCING)
			{
				ref_gimbal_yaw = current_middle_yaw + cmdYawGimbal;
				ref_gimbal_pitch = current_middle_pitch + cmdPitchGimbal;
			}
			else
			{
				if (zed_x_camera_sx != zed_x_camera_sx_prev || zed_y_camera_sx != zed_y_camera_sx_prev || zed_z_camera_sx != zed_z_camera_sx_prev) {
					
					// a new info just came from the stereocamera, so update the reference signal
					ref_gimbal_yaw = standard_gimbal.state_estim[0] + cmdYawGimbal;
					ref_gimbal_pitch = standard_gimbal.state_estim[1] + cmdPitchGimbal;
					ref_gimbal_pitch = 0;	// TODO delete
					
					// update the previous x,y,z values obtained from the stereocamera
					zed_x_camera_sx_prev = zed_x_camera_sx;
					zed_y_camera_sx_prev = zed_y_camera_sx;
					zed_z_camera_sx_prev = zed_z_camera_sx;
				}
			}
		}
		else if (robot_control_mode == CHASSIS_FOLLOW_GIMBAL_PC_MODE || robot_control_mode == FIXED_GIMBAL_PC_MODE) {		// Minecraft-like control with the mouse
			
			// next time that we switch to keyboard/mouse commands, we'll set a new starting position of the gimbal
			if (!need_to_set_initial_joystick_gimbal_position)
				need_to_set_initial_joystick_gimbal_position = 1;
			
			if (need_to_set_initial_mouse_gimbal_position) {
				
				cmdYawGimbal = standard_gimbal.state_estim[0];
				cmdPitchGimbal = standard_gimbal.state_estim[1];
				mouse_speed_x_prev = 0;
				mouse_speed_y_prev = 0;
				need_to_set_initial_mouse_gimbal_position = 0;
			}
			
			// this function uses 'mouse_meters_yaw_rad_ratio' and 'mouse_meters_pitch_rad_ratio' variables to convert from mouse's meters to gimbal's radiants
			update_mouse_position_gimbal(T_gimbal, &cmdYawGimbal, &cmdPitchGimbal);
		
			saturate(&cmdPitchGimbal, max_gimbal_pitch_inclination);		// saturation due to mechanical limit in pitch movements
			
			ref_gimbal_yaw = cmdYawGimbal;
			ref_gimbal_pitch = cmdPitchGimbal;
			
		}
	}
	else if (IS_SENTRY) {
		
		/* check if there're enemies in front of the sentry */
		zed_x_camera_sx = - get_AI_ref0();
		zed_y_camera_sx = get_AI_ref1();
		zed_z_camera_sx = - get_AI_ref2();
		if (zed_x_camera_sx == 0 && zed_y_camera_sx == 0 && zed_z_camera_sx == 0)		// based on what the stereocamera sees
			enemy_detected = 0;		// enemy not detected
		else
			enemy_detected = 1;		// enemy detected
		
		if (enemy_detected) {
			zed_x = zed_x_camera_sx + camera_sx_to_center_zed;
			zed_y = zed_y_camera_sx + center_zed_to_center_shooting;
			zed_z = zed_z_camera_sx;
			d = sqrt(zed_x*zed_x + zed_y*zed_y + zed_z*zed_z);
			if (zed_x_camera_sx == 0 && zed_y_camera_sx == 0 && zed_z_camera_sx == 0) {
				cmdPitchGimbal = 0;
				cmdYawGimbal = 0;
			}
			else {
				cmdPitchGimbal = atan(zed_y / sqrt(zed_y*zed_y + zed_z*zed_z));
				cmdYawGimbal = atan(zed_x / sqrt(zed_x*zed_x + zed_y*zed_y));
			}
			ref_gimbal_yaw = standard_gimbal.state_estim[0] + cmdYawGimbal;
			ref_gimbal_pitch = standard_gimbal.state_estim[1] + cmdPitchGimbal;
		}
		else {
			ref_gimbal_yaw = sentry_yaw_amplitude_scan_environment * sin(clock_ms*0.001*sentry_yaw_freq_scan_environment);
			ref_gimbal_pitch = sentry_pitch_amplitude_scan_environment * sin(clock_ms*0.001*sentry_pitch_freq_scan_environment);
		}
		
		// sentry cannot rotate yaw more than a certain value (because of limited cables' length)
		saturate(&ref_gimbal_yaw, 2*pi);
		
	}
	else
		return;
	
	
#if SYSID_GIMBAL
	ref_gimbal_pitch = -26*pi/180;
#endif
	
	standard_gimbal.ref[0] = ref_gimbal_yaw;
	standard_gimbal.ref[1] = ref_gimbal_pitch;
	standard_gimbal.ref[2] = 0;
	standard_gimbal.ref[3] = 0;
	
	
	// compute the control signals: standard_gimbal.control_signals[0] and [1] will be updated
	LQR_controller(STANDARD_GIMBAL);
	
	standard_gimbal.control_signals[0] = PID_control(&pid_yaw, 		standard_gimbal.error[0], standard_gimbal.integral_error[0], 	standard_gimbal.error[2],
																																UNDEF,										UNDEF,															UNDEF,
																																NULL,											NULL,																&fir_pid_D_yaw);
	standard_gimbal.control_signals[1] = PID_control(&pid_pitch, 	standard_gimbal.error[1], standard_gimbal.integral_error[1], 	standard_gimbal.error[3],
																																UNDEF,										UNDEF,															UNDEF,
																																&fir_pid_P_pitch,					NULL,																&fir_pid_D_pitch);
	
	//standard_gimbal.control_signals[1] = fir_filter(&fir_pid_pitch, standard_gimbal.control_signals[1]);

	
	
	// increase the gain if the gimbal has to move down (when it moves up it is already fast, because the center of mass is inclined backward)
	if (standard_gimbal.state_estim[1] > standard_gimbal.ref[1]) {
		standard_gimbal.control_signals[1] *= gain_pitch_goes_down;
		pitch_is_going_down = 1;
	}
	else
		pitch_is_going_down = 0;
	
	// increase gain (if needed) to mantain a certain orientation of the gimbal
	if (MANTAIN_POSITION_GIMBAL_GAIN) {
		
		decide_if_mantain_gimbal_position();		// this also (eventually) sets 'yaw_angle_where_mantain_position_starts' and 'pitch_angle_where_mantain_position_starts' 
		
		if (need_to_mantain_gimbal_yaw_position)
			standard_gimbal.control_signals[0] *= compute_gain_gimbal_mantain_position(YAW, yaw_angle_where_mantain_position_starts,
																																									length_yaw_gain_increase, max_gain_mantain_yaw, epsilon_yaw);
		if (need_to_mantain_gimbal_pitch_position)
			standard_gimbal.control_signals[1] *= compute_gain_gimbal_mantain_position(PITCH, pitch_angle_where_mantain_position_starts,
																																									length_pitch_gain_increase, max_gain_mantain_pitch, epsilon_pitch);
	}
	
	
	// make the gain proportional to the nearness of the gimbal to the reference angles
//	standard_gimbal.control_signals[0] *= exp_error_gain(standard_gimbal.error[0],
//																												0, middle_angle_exp_yaw, max_exp_gain_yaw_range,
//																												max_exp_yaw_error_gain, middle_value_exp_yaw, 1);

//	standard_gimbal.control_signals[1] *= exp_error_gain(standard_gimbal.error[1],
//																												0, middle_angle_exp_pitch, max_exp_gain_pitch_range,
//																												max_exp_pitch_error_gain, middle_value_exp_pitch, 1);
	
	// from torque to control signal (to be sent to the GM6020 motor)
	standard_gimbal.control_signals[0] *= gain_torque_to_control_signal_yaw;
	standard_gimbal.control_signals[1] *= gain_torque_to_control_signal_pitch;
	
#if SYSID_GIMBAL && !SYSID_RC
	standard_gimbal.control_signals[0] = send_data_sample(&gimbal_u_sysid);		// send signal to yaw motor (for system identification)
#endif
	
	// discrete-torque ratio of GM6020 motor
	standard_gimbal.control_signals[0] *= 1811.943;
	standard_gimbal.control_signals[1] *= 1811.943;
	
	// saturation of the control signals (in order to avoid possible damages)
	saturate(&standard_gimbal.control_signals[0], saturation_control_signal_yaw_gimbal);
	saturate(&standard_gimbal.control_signals[1], saturation_control_signal_pitch_gimbal);
	
	// [output] send the control signals
	if (IS_GIMBAL_ENABLED)
		CAN_cmd_gimbal((int16_t) standard_gimbal.control_signals[0], (int16_t) standard_gimbal.control_signals[1]);
	
	// set previous references, state estimations and errors for next iteration
	for (int i = 0; i < 4; i++) 
	{
		standard_gimbal.ref_prev[i] = standard_gimbal.ref[i];
		standard_gimbal.state_estim_prev[i] = standard_gimbal.state_estim[i];
		standard_gimbal.error_prev[i] = standard_gimbal.error[i];
	}
	
	// set previous flags for next iteration
	need_to_mantain_gimbal_yaw_position_prev = need_to_mantain_gimbal_yaw_position;
	need_to_mantain_gimbal_pitch_position_prev = need_to_mantain_gimbal_pitch_position;
	
	// set previous commands from joystick/mouse for next iteration
	cmdYawGimbal_prev = cmdYawGimbal;
	cmdPitchGimbal_prev = cmdPitchGimbal;
	
	
	/* acquire data for system identification */
#if SYSID_GIMBAL
	acquire_data_sample(&gimbal_yaw, 			standard_gimbal.y[0] * RAD_TO_DEG);
	acquire_data_sample(&gimbal_yaw_dot, 	standard_gimbal.y[2] * RAD_TO_DEG);
	acquire_data_sample(&gimbal_u_yaw, 		standard_gimbal.control_signals[0] * GM6020_DAC);
//	acquire_data_sample(&gimbal_ref_yaw, 	standard_gimbal.ref[0] * RAD_TO_DEG);
#endif
}


void gimbal_control_init(void) {
	
	gimbal_model_init(&standard_gimbal);			// initialization of the model for the gimbal
	LQR_gain_init(STANDARD_GIMBAL);						// initialization of the LQR gain for the gimbal
	saturation_control_signal_yaw_gimbal = MAX_GIMBAL_YAW_CONTROL_SIGNAL_AMPLITUDE;
	saturation_control_signal_pitch_gimbal = MAX_GIMBAL_PITCH_CONTROL_SIGNAL_AMPLITUDE;
	
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
		gimbal_u_sysid.samples[i] *= (saturation_control_signal_yaw_gimbal / max_abs_u) * gain_u_sysid;		// TODO delete this comment: the "2" is just to stimulate the entire yaw dynamics
	}

#endif
}


