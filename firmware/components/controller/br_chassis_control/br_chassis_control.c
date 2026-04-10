#include <stdint.h>
#include "br_chassis_control.h"
#include "control_alg.h"
#include "control_util.h"
#include "CAN_receive.h"
#include "bmi088driver.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "robot_config.h"
#include "referee_alg.h"
#include "chassis_task.h"


/**************************************************************
	NOTES about the Balancing Robot:
	- The state variables of the Balancing Robot are ...
			0->pos_left_wheel, 1->pos_dot_left_wheel, 2->pos_right_wheel, 3->pos_dot_right_wheel, 4->pitch, 5->pitch_dot (it's important to mantain this order)
		... which are:
			0->linear position of left wheel, 1->linear velocity of left wheel,
			2->linear position of right wheel, 3->linear velocity of right wheel,
			4->angular position of the chassis, 5->angular velocity of the chassis
**************************************************************/


/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** robot model and physical parameters ***/
struct robot br_chassis;		// TODO put this as static (when gimbal code will be refactored)
static uint8_t n = 9;									// number of robot states
static uint8_t m = NUM_BR_WHEELS;			// number of robot inputs
static float wheels_radius = 0.09;		// radius of the wheels [m]
static float chassis_width = 0.4;			// width of balancing robot chassis (i.e. distance between the 2 wheels) [m]


/*** controllers ***/
// LQR for angular position and velocity
static struct LQR lqr;
static float K[] 	=	{0.0,	0.0,	0.0,	0.0,	1.62,	0.135,	0.0,	0.0,	0.0,
										 0.0,	0.0,	0.0,	0.0,	1.62,	0.135,	0.0,	0.0,	0.0};
static float Ki[] = {0.0,	0.0,	0.0,	0.0,	0.0,	0.0,		0.0,	0.0,	0.0,
										 0.0,	0.0,	0.0,	0.0,	0.0,	0.0,		0.0,	0.0,	0.0};
// PID used to modify theta reference in order to break linear movements
static struct PID_ pid_break_lin_mov;
static float Kp_break_lin_mov = 0.0;
static float Ki_break_lin_mov = 0.0;
static float Kd_break_lin_mov = 0.3;
// PID to rotate chassis
static struct PID_ pid_rot;
static float Kp_rot = 3;
static float Ki_rot = 0.0;
static float Kd_rot = 0.0;
// PID to control wheels linear velocity (not very useful)
static struct PID_ pid_lin_vel;
static float Kp_lin_vel = 0.01;
static float Ki_lin_vel = 0.0;
static float Kd_lin_vel = 0.0;
// closed-loop PID control on wheels motors
static struct PID_ pid_wheel[2];
static float Kp_wheel[] = {6.0, 		6.0};			//4
static float Ki_wheel[] = {30.0, 		30.0};		//30
static float Kd_wheel[] = {11.0, 		11.0};		//3
static float Kp_wheel_ = 4;
static float Ki_wheel_ = 30;
static float Kd_wheel_ = 3;
static float lin_pos_error_bounds[] = {0.06,	0.8};
static struct fir fir_pid_D_wheel[2];
static uint32_t fir_pid_D_wheel_len = 2;
static float fir_pid_D_wheel_coeff[] = {0.8, 0.2};
/*
static uint32_t fir_pid_D_wheel_len = 31;
static float fir_pid_D_wheel_coeff[] = {-0.000043875672509666, -0.000256268286159164, 0.000584614550591803, 0.001206506631839313, -0.002488211113565625, -0.003219648514932325, 0.007421771098377786, 0.006517015611710641, -0.018252734512607401, -0.010834123509247253, 0.040393167853404709, 0.015308168143109252, -0.090291146538250197, -0.018720143197473602, 0.312684085827697245, 0.519981643256028914, 0.312684085827697300, -0.018720143197473602, -0.090291146538250211, 0.015308168143109258, 0.040393167853404709, -0.010834123509247262, -0.018252734512607414, 0.006517015611710644, 0.007421771098377786, -0.003219648514932324, -0.002488211113565629, 0.001206506631839314, 0.000584614550591803, -0.000256268286159165, -0.000043875672509667};
*/



// PID control to compute theta reference when there are commands from the remote controller (to move forward/backward)
static struct PID_ pid_ref_theta_rc;
static float Kp_ref_theta_rc = 0.4;
static float Ki_ref_theta_rc = 0.0;
static float Kd_ref_theta_rc = 0.0;


/*** control parameters ***/
// pilot linear movements on ground
static float cmd_fwd_bwd					= 0;			// forward/backward command sent by the pilot (either through remote controller or keyboard)
static float cmd_right_left				= 0;			// right/left command sent by the pilot (either through remote controller or keyboard)
static float max_ref_pos_dot_rc 	= 1.0;		// max reference for wheels linear velocity obtainable by the remote controller		[m/s]
// keep constant COM linear velocity
static float sat_ref_theta_cmd[] = {7*pi/180, 10*pi/180};						// range for adaptative saturation (starting from COM) of theta reference given by a linear velocity reference (generated through remote commands)	[rad]
static float pos_dot_com_sat_ref_theta_bounds[] = {0.05, 0.2};			// COM linear velocity bounds in which the adaptative saturation of theta reference (generated through remote commands) varies	[m/s]
static float sat_ref_theta_physical_limit = 13*pi/180;							// saturation (starting from 0 angle) of theta reference given by the physical limitations on the robot inclination (to avoid touching the ground)	[rad]
// autonomous break of COM linear velocity
static float ref_theta_com_before_brake;								// value of the theta reference determined by the COM shift immediatly before the trigger of braking mode		[rad]
static int sign_com_lin_vel;														// stores the sign (i.e. direction) of COM linear velocity immediatly before the trigger of braking mode
static float brake_trigger_com_lin_vel 	= 0.4;					// COM linear velocity that triggers the braking mode (in absence of commands from the remote controller)		[m/s]
static float sat_pid_P_break_lin_mov 		= 2*pi/180;			// saturation of PID Proportional control signal used to automatically break COM linear velocity	[rad]
static float sat_pid_D_break_lin_mov 		= 10*pi/180;		// saturation of PID Derivative control signal used to automatically break COM linear velocity		[rad]
static float pos_com_before_brake;											// TODO check if this is actually used
// closed-loop control on wheels motors
static float sat_integr_error_pos = 0.3;			// saturation of linear position error integral of each wheel		[m*s]
static float max_lin_pos = 1.0;								// max linear position reference that can be set (starting from the current linear position) 		[m]
// adaptive control for COM (Center of Mass) shift
static float ref_theta_com = 0;								// component of theta reference determined by the COM shift																									[rad]
static float gain_theta_com = 0.5;						// gain the determines how fast the COM shift influences the reference of theta
static float sat_theta_com = 12*pi/180;				// saturation value for the theta reference component determined by the COM shift														[rad]
// unstuck from wall
static unsigned long clock_tick_ms_before_unstuck_wall;
static unsigned long clock_tick_ms_while_unstuck_wall;
static float theta_before_unstuck_wall;
static float theta_tol_unstuck_wall = 3*pi/180;					// [rad]
static float chassis_accel_double_integr = 0;						// [m]
static float chassis_accel_double_integr_tol = 0.1;			// [m]
static float time_before_unstuck_wall = 1.0;										// [s]
static float time_while_unstuck_wall = 0.75;										// [s]
static float ref_pos_unstuck_wall = 0.2;
static int sign_ref_pos_unstuck_wall;
static float u_trigger_unstuck_wall = 5000;						// discrete control signal on wheels that must be kept (for a certain time) in order to trigger the "unstuck from wall" routine

/*** gains to be tuned ***/
static float gain_lqr_no_remote_commands 			= 1.3;			// gain of LQR output when there're no remote commands
static float gain_unstuck_from_wall 					= 20.0;			// gain of open-loop command used to unstuck from wall
static float gain_overall_u 									= 9.765;		// overall gain of balancing robot chassis
static float gain_ref_theta_pos_dot_com				= 0.3;			// gain that penalizes high errors in the COM linear velocity by inclining more the angular position (to both brake and accelerate)
static float gain_rot 												= 1;				// gain to rotate chassis
static float gain_lin_vel 										= 1;				// gain to individually control wheels linear velocity (not very useful)
static float gain_pid_P_wheels_unstuck_wall 	= 1.5;			// gain of PID Proportional control signal of wheels to unstuck from wall
static float gain_pid_I_wheels_unstuck_wall 	= 2.5;			// gain of PID Integrative control signal of wheels to unstuck from wall
	

/*** flags ***/
static int is_first_iter 							= 1;		// true if the current algorithm iteration is the first one
static int should_rotate_chassis 			= 0;		// true if chassis has to rotate contiguously (defense mode)
static int should_unstuck_from_wall 	= 0;		// true if the balancing robot has to unstuck from a wall
static int is_90deg_mode_active 			= 0;		// true if balancing robot is in 90 degree mode (i.e. chassis mantains an angle of 90 degrees w.r.t. the gimbal)
static int is_brake_mode_active 			= 0;		// true if balancing robot has to brake its linear velocity (automatically, without any command from the remote controller)


/* system identification */
#if SYSID_BR_CHASSIS
struct data_sample br_chassis_theta;
struct data_sample br_chassis_theta_dot;
struct data_sample br_chassis_pos_dot;
struct data_sample br_chassis_u;
#endif


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void br_chassis_control_loop()
{

	if (is_first_iter)
	{	
		/* initialize the balancing robot chassis data structure */
		robot_init(&br_chassis, n, m);
		
		for (int wheel = 0; wheel < NUM_BR_WHEELS; wheel++)
			fir_init(&fir_pid_D_wheel[wheel], fir_pid_D_wheel_len, fir_pid_D_wheel_coeff);
		
		/* initialize the controllers */
		
		// LQR that, given angular position and velocity, computes the reference position of the wheels
		LQR_init(&lqr, K, Ki, m, n);
		// PID that computes a (temporary) reference angular position in order to break the unwanted linear velocities
		PID_init_(&pid_break_lin_mov, Kp_break_lin_mov, Ki_break_lin_mov, Kd_break_lin_mov, dt_chassis);
		// PID used for chassis-follow-gimbal
		PID_init_(&pid_rot, Kp_rot, Ki_rot, Kd_rot, dt_chassis);
		// PID used to independently control the linear velocity of each wheel (not very useful)
		PID_init_(&pid_lin_vel, Kp_lin_vel, Ki_lin_vel, Kd_lin_vel, dt_chassis);
		// PID that computes the theta reference when there are commands from the remote controller (to move forward/backward)
		PID_init_(&pid_ref_theta_rc, Kp_ref_theta_rc, Ki_ref_theta_rc, Kd_ref_theta_rc, dt_chassis);
		
		// initialize system identification data structures
#if SYSID_BR_CHASSIS
		data_sampling_init(&br_chassis_theta, 			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_theta_dot, 	SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_pos_dot, 		SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_u, 					SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
#endif
	}
	
	
	// check keyboard commands for balancing robot
//	check_keyboard_commands_balancing(robot_control_mode);	// TODO refactor this function
	
	
	/* take relative Yaw position of chassis w.r.t. gimbal */
	br_chassis.x[6] = ((float) (((int) ((get_yaw_gimbal_motor_measures()->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET) * 180/pi)) % 360) * pi/180);
	/* NOTE: the "... % 360" is used to avoid doing more than 360 deg to align chassis with gimbal */
	
	// trasformation of angle range from 0/+360 to -180/+180 deg
	if (br_chassis.x[6] > 180)
		br_chassis.x[6] -= 360;
	else if (br_chassis.x[6] < -180)
		br_chassis.x[6] += 360;
	/* NOTE: the trasformation from 0/+360 to -180/+180 guarantees that,
			in order to realign towards the gimbal, the chassis takes the shortest path (clockwise or counter-clockwise) */
	
	// check if we need to rotate chassis to 90 deg
	if (robot_control_mode == REMOTE_CONTROLLER_MODE)
		is_90deg_mode_active = switch_is_mid(rc_left_switch);
	else
		is_90deg_mode_active = 0;
	
	
	// collect data about current robot's state from encoder and IMU
	br_chassis.x[0] = (float)((get_chassis_motor_measures(0)->contig_ang_pos_rad) * wheels_radius);
	br_chassis.x[1] = (float)((get_chassis_motor_measures(0)->ang_vel_radsec - gx) * wheels_radius);
	br_chassis.x[2] = (float)((- get_chassis_motor_measures(1)->contig_ang_pos_rad) * wheels_radius);
	br_chassis.x[3] = (float)((- get_chassis_motor_measures(1)->ang_vel_radsec - gx) * wheels_radius);
	br_chassis.x[4] = (3.1415/180)*ins_correct_angle[0];
	br_chassis.x[5] = gx;
	br_chassis.x[7] = (br_chassis.x[0] + br_chassis.x[2]) / 2;
	br_chassis.x[8] = (br_chassis.x[1] + br_chassis.x[3]) / 2;
	
	
	if (is_first_iter) {
		br_chassis.ref[0] = br_chassis.x[0];
		br_chassis.ref[2] = br_chassis.x[2];
	}

	// collect commands sent by means the remote controller
	if (robot_control_mode == REMOTE_CONTROLLER_MODE) {
		cmd_fwd_bwd = (float) get_remote_control_point()->rc.ch[3]; 	// left stick, up/down movement
		cmd_right_left = (float) get_remote_control_point()->rc.ch[2];	// left stick, left/right movement
	}
	else if (robot_control_mode == CHASSIS_FOLLOW_GIMBAL_PC_MODE || robot_control_mode == BALANCING_90_DEGREE_PC_MODE) {
		compute_weights_WASD_keys(dt_chassis);
		cmd_fwd_bwd = 500.0 * weight_fwd_key;
		cmd_fwd_bwd -= 500.0 * weight_bwd_key;
		cmd_right_left = 500.0 * weight_right_key;
		cmd_right_left -= 500.0 * weight_left_key;
		
		// this is to make the robot accelerate faster
		cmd_fwd_bwd *= 1.5;
		cmd_right_left *= 1.5;
		saturate(&cmd_fwd_bwd, 500.0);
		saturate(&cmd_right_left, 500.0);
	}
	
	
	
	/*** unstuck from wall ***/
	
	// if this is first algorithms iteration, start the routine to detect if the robot is stuck on wall
	if (is_first_iter) {
		clock_tick_ms_before_unstuck_wall = HAL_GetTick();
		chassis_accel_double_integr = 0;
		theta_before_unstuck_wall = br_chassis.x[4];
	}
	
	/* check if we should unstuck from wall */
	if (!should_unstuck_from_wall) {
		
		// conditions to be satisfied (all together, for a certain time interval) in order to trigger the "unstuck from wall" routine
		if (fabs(br_chassis.u[0]) >= u_trigger_unstuck_wall &&
				fabs(br_chassis.u[1]) >= u_trigger_unstuck_wall &&
				sign_zero_undef(br_chassis.u[0]) != sign_zero_undef(br_chassis.u[1]) &&
				fabs(br_chassis.x[4] - theta_before_unstuck_wall) <= theta_tol_unstuck_wall &&
				fabs(chassis_accel_double_integr) < chassis_accel_double_integr_tol) {
			
			if (HAL_GetTick() - clock_tick_ms_before_unstuck_wall >= time_before_unstuck_wall*1e3) {
				
				// trigger the "unstuck from wall" routine
				clock_tick_ms_while_unstuck_wall = HAL_GetTick();
				sign_ref_pos_unstuck_wall = - sign_zero_undef(br_chassis.u[0]);
				should_unstuck_from_wall = 1;
			}
			else {
				
				// continue to compute the double integral of microprocessor's acceleration
				chassis_accel_double_integr += (1/2)*(ay*cos(br_chassis.x[4]))*dt_chassis*dt_chassis;
			}
		}
		else {
			
			// reset all the conditions needed to trigger the "unstuck from wall" routine
			clock_tick_ms_before_unstuck_wall = HAL_GetTick();
			chassis_accel_double_integr = 0;
			theta_before_unstuck_wall = br_chassis.x[4];
		}
	}
	
	/* check if we completed the "unstuck from wall" routine */
	if (should_unstuck_from_wall && HAL_GetTick() - clock_tick_ms_while_unstuck_wall >= time_while_unstuck_wall*1e3) {
		
		// disable the "unstuck from wall" routine
		clock_tick_ms_before_unstuck_wall = HAL_GetTick();
		chassis_accel_double_integr = 0;
		theta_before_unstuck_wall = br_chassis.x[4];
		should_unstuck_from_wall = 0;
		
		// reset the integral error cumulated by both wheels' position (to avoid too high control signals when rising up from the ground)
		reset_integr_error_if_overshoot(&br_chassis, 0);
		reset_integr_error_if_overshoot(&br_chassis, 2);
	}
	
	
	
	
	
#if !IS_90DEG_MODE_ENABLED
	is_90deg_mode_active = 0;
#endif
	
#if !IS_CHASSIS_ROT_MODE_ENABLED
	should_rotate_chassis = 0;
#endif
	
#if !IS_UNSTUCK_FROM_WALL_MODE_ENABLED
	should_unstuck_from_wall = 0;
#endif
	
	
	/* define and acquire reference signals (depending on the current situation) */
	float ref_theta_break_lin_mov;
	float ref_pos_dot_90deg;						// reference of linear velocity on ground (of each wheel) used to go to 90 degree mode
	

	if ((cmd_fwd_bwd == 0 && cmd_right_left == 0) || should_rotate_chassis) {
		
		br_chassis.ref[1] = 0;
		br_chassis.ref[3] = 0;
		br_chassis.ref[4] = 0;
		br_chassis.ref[5] = 0;
		br_chassis.ref[8] = 0;
		
		// compute COM linear velocity error
		ref_error_state(&br_chassis, 8, dt_chassis);
		reset_integr_error_if_overshoot(&br_chassis, 8);
		
		/* adapt balance point to COM shift */
		ref_theta_com += dt_chassis * br_chassis.x[8] * gain_theta_com;
		saturate(&ref_theta_com, sat_theta_com);
		
		ref_theta_break_lin_mov = - PID_control(&pid_break_lin_mov,
																						br_chassis.error[7],			br_chassis.integr_error[7], br_chassis.error[8],
																						sat_pid_P_break_lin_mov,	0.0,												sat_pid_D_break_lin_mov,
																						NULL,											NULL,												NULL);
		
		/* activate braking mode to brake COM linear velocity (if it is going too fast) */
//		if (is_first_iter)
//			pos_com_before_brake = br_chassis.x[7];
//		
//		if (!is_brake_mode_active && (fabs(br_chassis.x[8]) >= brake_trigger_com_lin_vel)) {
//			
//			if (fabs(br_chassis.x[7] - pos_com_before_brake) >= 0.25) {
//				
//				sign_com_lin_vel = sign_zero_undef(br_chassis.x[8]);		// store sign of COM linear velocity just before the trigger of braking mode
//				ref_theta_com_before_brake = ref_theta_com;							// store COM equilibrium angle just before the trigger of braking mode
//				is_brake_mode_active = 1;
//			}
//		}
//		else {
//			pos_com_before_brake = br_chassis.x[7];
//		}
		if (!is_brake_mode_active && (fabs(br_chassis.x[8]) >= brake_trigger_com_lin_vel)) {
			
			sign_com_lin_vel = sign_zero_undef(br_chassis.x[8]);		// store sign of COM linear velocity just before the trigger of braking mode
			ref_theta_com_before_brake = ref_theta_com;							// store COM equilibrium angle just before the trigger of braking mode
			is_brake_mode_active = 1;																// trigger braking mode
		}
		
		/* deactivate braking mode (if we managed to brake the COM linear velocity) */
		if (is_brake_mode_active && (sign_zero_undef(br_chassis.x[8]) != sign_com_lin_vel)) {
			
			ref_theta_com = ref_theta_com_before_brake;							// resume COM equilibrium angle that there was before braking mode
			is_brake_mode_active = 0;																// trigger braking mode
		}
		
		
		// theta
		br_chassis.ref[4] += ref_theta_com;
		br_chassis.ref[4] += ref_theta_break_lin_mov;
		saturate(&br_chassis.ref[4], sat_ref_theta_physical_limit);
		
		// pos_dot of wheels
		br_chassis.ref[1] = br_chassis.x[1];
		br_chassis.ref[3] = br_chassis.x[3];
	
	}
	else {		// there are commands from keyboard/remote controller
		
		/*** definition of variables ***/
		float cmd_rc;										// command received from remote controller (in range [-660, +660]
		float ref_theta_pos_dot_com;		// component of theta reference (starting at the COM) that depends on the COM linear velocity error w.r.t. the reference
		float sat_ref_theta;						// saturation for theta reference
		
		/*** reference of theta dot ***/
		br_chassis.ref[5] = 0;
		
		
		/*** reference of theta ***/
		// get COM linear velocity reference from remote controller
		cmd_rc = (!is_90deg_mode_active) ? cmd_fwd_bwd : cmd_right_left;
		br_chassis.ref[8] = (cmd_rc / MAX_RC_TILT) * (max_ref_pos_dot_rc);		// reference of COM linear velocity (corresponding to the remote controller command)
		
		// compute COM linear velocity error
		ref_error_state(&br_chassis, 8, dt_chassis);
		
		// account for situations where remote controller commands are being sent when robot is in brake mode
		if (is_brake_mode_active) {
			ref_theta_com = ref_theta_com_before_brake;
			is_brake_mode_active = 0;
		}
		
		// compute theta reference
		br_chassis.ref[4] = ref_theta_com;			// make theta reference start at the COM equilibrium point
		
		ref_theta_pos_dot_com = PID_control(&pid_ref_theta_rc,
																				br_chassis.error[8],	br_chassis.integr_error[8], 0 - ay,			// compute theta reference (from COM equilibrium point) with a PID
																				UNDEF,														UNDEF,											UNDEF,
																				NULL,															NULL,												NULL);
		ref_theta_pos_dot_com *= (1 + fabs(br_chassis.x[8]) * gain_ref_theta_pos_dot_com);		// penalize high COM linear velocities by inclining more theta reference
		
		// saturate theta reference
		sat_ref_theta = range_lin_prop(fabs(br_chassis.x[8]), pos_dot_com_sat_ref_theta_bounds[0], pos_dot_com_sat_ref_theta_bounds[1], sat_ref_theta_cmd[0], sat_ref_theta_cmd[1]);
		saturate(&ref_theta_pos_dot_com, sat_ref_theta);								// saturation from COM (which value depends on the COM linear velocity)
		br_chassis.ref[4] -= ref_theta_pos_dot_com;
		saturate(&br_chassis.ref[4], sat_ref_theta_physical_limit);			// saturation from 0 angle of board (due to physical limitations to avoid to touch the ground)
	}
	
	/* set reference for chassis orientation (chassis-follow-gimbal, 90 degree mode or rotating chassis mode) */
	if (is_90deg_mode_active)
		br_chassis.ref[6] = pi/2;
	else
		br_chassis.ref[6] = 0;
	

	
	/*** control algorithms ***/
	
	// reset the integral error area if the reference have been overshot
	for (int state = 0; state < n; state++) {
		
		if (state == 0 || state == 2 || state == 8)
			continue;
		
		ref_error_state(&br_chassis, state, dt_chassis);
		reset_integr_error_if_overshoot(&br_chassis, state);
	}
	
	// LQR control for angular position and velocity
	LQR_control(&br_chassis, &lqr);
	
	if (cmd_fwd_bwd == 0 && cmd_right_left == 0) {
		for (int wheel = 0; wheel < m; wheel++) {
			br_chassis.u[wheel] *= gain_lqr_no_remote_commands;
		}
	}
	
	// saturation of LQR output (to avoid extremely high values)
	for (int wheel = 0; wheel < m; wheel++) {
		saturate(&br_chassis.u[wheel], MAX_BR_LQR_THETA_OUT);
	}

	
	/* closed-loop PID for left and right wheel */
	{
		int state = 0;
		int output = 0;
		
		// for each wheel...
		while (state == 0 || state == 2) {
			
			/* PID control of a wheel */
			float ref_offset_pos;
			
			if (!should_unstuck_from_wall)
				ref_offset_pos = br_chassis.u[output] * max_lin_pos;
			else
				ref_offset_pos = ref_pos_unstuck_wall * sign_ref_pos_unstuck_wall;
			
			br_chassis.ref[state] = br_chassis.x[state] + ref_offset_pos;
			ref_error_state(&br_chassis, state, dt_chassis);
			saturate(&br_chassis.integr_error[state], sat_integr_error_pos);
			
			float Kp 	= range_lin_prop(br_chassis.error[state], lin_pos_error_bounds[0], lin_pos_error_bounds[1], Kp_wheel[0], 	Kp_wheel[1]);
			float Ki 	= range_lin_prop(br_chassis.error[state], lin_pos_error_bounds[0], lin_pos_error_bounds[1], Ki_wheel[0], 	Ki_wheel[1]);
			float Kd 	= range_lin_prop(br_chassis.error[state], lin_pos_error_bounds[0], lin_pos_error_bounds[1], Kd_wheel[0], 	Kd_wheel[1]);
			
			if (should_unstuck_from_wall) {
				Kp *= gain_pid_P_wheels_unstuck_wall;
				Ki *= gain_pid_I_wheels_unstuck_wall;
			}
			
			Kp = Kp_wheel_;
			Ki = Ki_wheel_;
			Kd = Kd_wheel_;
			
			PID_init_(&pid_wheel[output], Kp, Ki, Kd, dt_chassis);
			br_chassis.u[output] = PID_control(&pid_wheel[output],
																				br_chassis.error[state],	br_chassis.integr_error[state], 0 - br_chassis.x[state+1],
																				UNDEF,										UNDEF,													UNDEF,
																				NULL,											NULL,														NULL/*&fir_pid_D_wheel[output]*/);
			// update counters
			state += 2;
			output += 1;
		}
	}
	

	/* rotate chassis (to follow gimbal or to go 90 degrees mode) */
	float u_rot_chassis = PID_control(&pid_rot,
																		br_chassis.error[6],	br_chassis.integr_error[6], br_chassis.deriv_error[6],
																		UNDEF,								UNDEF,											UNDEF,
																		NULL,									NULL,												NULL);
	u_rot_chassis *= gain_rot;
	
	/* control wheels linear velocity */
	float u_pos_dot_L = gain_lin_vel * PID_control(&pid_lin_vel,
																								br_chassis.error[1],	br_chassis.integr_error[1], br_chassis.deriv_error[1],
																								UNDEF,								UNDEF,											UNDEF,
																								NULL,									NULL,												NULL);
	float u_pos_dot_R = gain_lin_vel * PID_control(&pid_lin_vel,
																								br_chassis.error[3],	br_chassis.integr_error[3], br_chassis.deriv_error[3],
																								UNDEF,								UNDEF,											UNDEF,
																								NULL,									NULL,												NULL);
	
	if (!should_unstuck_from_wall) {
	
		/* sum up control signals of individual tasks to the overall control signal */
		br_chassis.u[0] += + u_rot_chassis + u_pos_dot_L;
		br_chassis.u[1] += - u_rot_chassis + u_pos_dot_R;
	}
	
	// right wheel has opposite sign w.r.t. the left wheel
	br_chassis.u[1] *= -1;
	
	/* manage power consumption */
#if IS_POW_CONS_CHASSIS_ENABLED
		check_chassis_power_consumption();
#endif
	
	/* acquire data for system identification */
#if SYSID_BR_CHASSIS
	acquire_data_sample(&br_chassis_theta, 					br_chassis.x[4]);
	acquire_data_sample(&br_chassis_theta_dot, 			br_chassis.x[5]);
	acquire_data_sample(&br_chassis_pos_dot, 				br_chassis.x[7]);
	acquire_data_sample(&br_chassis_u, 							(br_chassis.u[0] + br_chassis.u[1]) / 2);
#endif
	
	/* analog-to-digital convertion + saturation of chassis motors commands */
	for (int wheel = 0; wheel < m; wheel++) {
		br_chassis.u[wheel] *= gain_overall_u;
		br_chassis.u[wheel] *= M3508_C620_ADC;
		saturate(&br_chassis.u[wheel], MAX_BR_CHASSIS_OVERALL_U);
	}
	
	/* send control signals to chassis motors */
	if (IS_CHASSIS_ENABLED)
		CAN_cmd_chassis((int16_t) br_chassis.u[0],
										(int16_t) br_chassis.u[1],
										(int16_t) 0,
										(int16_t) 0);
	
	/* save the robot state of the current iteration, in order to use it for the next iteration */
	save_prev_robot_state(&br_chassis);
	
	// first algorithm iteration terminates here
	is_first_iter = 0;
}



