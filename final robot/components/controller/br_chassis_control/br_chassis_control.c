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
#include "kalman_filter.h"


/**************************************************************
	NOTES about the Balancing Robot:
	- The state variables of the Balancing Robot are ...
			0->pos_left_wheel, 1->pos_dot_left_wheel, 2->pos_right_wheel, 3->pos_dot_right_wheel, 4->pitch, 5->pitch_dot (it's important to mantain this order)
		... which are:
			0->linear position of left wheel, 1->linear velocity of left wheel,
			2->linear position of right wheel, 3->linear velocity of right wheel,
			4->angular position of the chassis, 5->angular velocity of the chassis
**************************************************************/

// Number of states, inputs and outputs (and their multiplications) for the Extended Kalman Filter (EKF)
#define EKF_n 6
#define EKF_m 2
#define EKF_q 6
#define EKF_nn 36
#define EKF_qq 36
#define EKF_nq 36
#define EKF_qn 36

/*** robot model and physical parameters ***/
struct robot br_chassis;		// TODO put this as static (when gimbal code will be refactored)
static uint8_t n = 9;									// number of robot states
static uint8_t m = NUM_BR_WHEELS;			// number of robot inputs
static float wheels_radius = 0.09;		// radius of the wheels [m]
static float chassis_width = 0.4;			// width of balancing robot chassis (i.e. distance between the 2 wheels) [m]


/*** controllers ***/
// LQR for angular position and velocity
static struct LQR lqr;
static float K_lqr[] 	=	{0.0,	0.0,	0.0,	0.0,	1.62,	0.135,	0.0,	0.0,	0.0,
										 0.0,	0.0,	0.0,	0.0,	1.62,	0.135,	0.0,	0.0,	0.0};
static float Ki_lqr[] = {0.0,	0.0,	0.0,	0.0,	0.0,	0.0,		0.0,	0.0,	0.0,
										 0.0,	0.0,	0.0,	0.0,	0.0,	0.0,		0.0,	0.0,	0.0};
// PID used to modify theta reference in order to break linear movements
static struct PID_ pid_break_lin_mov;
static float Kp_break_lin_mov = 0.0;
static float Ki_break_lin_mov = 0.0;
static float Kd_break_lin_mov = 0.3;
// PID to rotate chassis
static struct PID_ pid_rot;
static float Kp_rot = 6.0;
static float Ki_rot = 1.0;
static float Kd_rot = 0.0;
// PID to control wheels linear velocity
static struct PID_ pid_lin_vel;
static float Kp_lin_vel = 0.0;
static float Ki_lin_vel = 0.0;
static float Kd_lin_vel = 0.0;
// PID to control unstuck from wall
static struct PID_ pid_unstuck_wall;
static float Kp_unstuck_wall = 5.0;
static float Ki_unstuck_wall = 0.0;
static float Kd_unstuck_wall = 0.0;
// closed-loop PID control on wheels motors
static struct PID_ pid_wheel;
static float Kp_wheel = 6.0;
static float Ki_wheel = 30.0;
static float Kd_wheel = 5.0;
// PID control to compute theta reference when there are commands from the remote controller (to move forward/backward)
static struct PID_ pid_ref_theta_rc;
static float Kp_ref_theta_rc = 0.4;
static float Ki_ref_theta_rc = 0.0;
static float Kd_ref_theta_rc = 0.0;


/*** filters ***/
// filter for wheels motors
static struct fir fir_pid_D_wheel[2];
static uint32_t fir_pid_D_wheel_len = 2;
static float fir_pid_D_wheel_coeff[] = {0.8, 0.2};


/*** control parameters ***/
// pilot linear movements on ground
static float cmd_fwd_bwd					= 0;			// forward/backward command sent by the pilot (either through remote controller or keyboard)
static float cmd_right_left				= 0;			// right/left command sent by the pilot (either through remote controller or keyboard)
static float max_ref_pos_dot_rc 	= 1.2;		// max reference for wheels linear velocity obtainable by the remote controller		[m/s]
// keep constant COM linear velocity
static float sat_ref_theta_cmd[] = {7*pi/180, 10*pi/180};						// range for adaptative saturation (starting from COM) of theta reference given by a linear velocity reference (generated through remote commands)	[rad]
static float pos_dot_com_sat_ref_theta_bounds[] = {0.05, 0.2};			// COM linear velocity bounds in which the adaptative saturation of theta reference (generated through remote commands) varies	[m/s]
static float sat_ref_theta_physical_limit = 15*pi/180;							// saturation (starting from 0 angle) of theta reference given by the physical limitations on the robot inclination (to avoid touching the ground)	[rad]
// autonomous break of COM linear velocity
static float theta_com_before_brake;													// value of the theta COM determined by the COM shift immediatly before the trigger of braking mode		[rad]
static int sign_com_lin_vel;																	// stores the sign (i.e. direction) of COM linear velocity immediatly before the trigger of braking mode
static float brake_trigger_com_lin_vel 				= 0.2;					// COM linear velocity that triggers the braking mode (in absence of commands from the remote controller)		[m/s]
static float brake_trigger_lin_pos 						= 0.1;					// elapsed COM linear distance that triggers the braking mode (in absence of commands from the remote controller)		[m]
static float sat_pid_P_break_lin_mov 					= 2*pi/180;			// saturation of PID Proportional control signal used to automatically break COM linear velocity	[rad]
static float sat_pid_D_break_lin_mov 					= 10*pi/180;		// saturation of PID Derivative control signal used to automatically break COM linear velocity		[rad]
static float pos_com_before_brake;														// TODO check if this is actually used
static float theta_com_brake;
static float tol_lin_vel_com_exit_brake 			= 999.0;				// tolerance for linear COM velocity to exit the brake mode (at least 0.8 m/s)		[m/s]
static unsigned long clock_tick_ms_last_brake	= 0;						// clock tick of the last (i.e., most recent) brake		[ms]
static float min_time_between_brakes 					= 1.2;					// minimum time that must elapse between two consecutive brakes		[s]
static float max_time_in_brakes								= 0.2;					// maximum time that can elapse between two consecutive "brake mode" activations within the same brake maneuver		[s]
// closed-loop control on wheels motors
static float sat_integr_error_pos = 0.3;			// saturation of linear position error integral of each wheel		[m*s]
static float max_lin_pos = 1.0;								// max linear position reference that can be set (starting from the current linear position) 		[m]
static float gain_pos_brake = 0.00274;
static float gain_pos_brake_theta = 200;
static float min_ref_pos_brake = 0.1;
// adaptive control for COM (Center of Mass) shift
static float theta_com = 0;								// component of theta reference determined by the COM shift																									[rad]
static float gain_theta_com = 0.5;						// gain the determines how fast the COM shift influences the reference of theta
static float sat_theta_com = 12*pi/180;				// saturation value for the theta reference component determined by the COM shift														[rad]
// unstuck from wall
static unsigned long clock_tick_ms_before_unstuck_wall;
static unsigned long clock_tick_ms_while_unstuck_wall;
static float theta_before_unstuck_wall;
static float theta_tol_unstuck_wall 						= 6*pi/180;					// [rad]
static float chassis_accel_double_integr 				= 0;								// [m]
static float chassis_accel_double_integr_tol 		= 0.4;							// [m]
static float time_before_unstuck_wall 					= 1.0;							// [s]
static float time_while_unstuck_wall 						= 1.1;							// overall time duration of the "unstuck from wall" routine (when the chassis motors first shut down, and then move in the opposite direction w.r.t. the wall)		[s]
static float time_shut_down_unstuck_wall 				= 0.5;							// time duration at the beginning of the "unstuck from wall" routine in which the chassis motors shut down	[s]
static float ref_pos_dot_unstuck_wall 					= 0.8;							// [m/s]
static int sign_ref_pos_dot_unstuck_wall;
static float u_trigger_unstuck_wall 						= 5000;						// discrete control signal on wheels that must be kept (for a certain time) in order to trigger the "unstuck from wall" routine
static float theta_com_before_unstuck_wall;
// chassis rotation
static float sat_pid_rot = 6.0;		// 4.25
static float time_ms_begin_rot;
static float chassis_yaw_begin_rot;
static float gimbal_yaw_begin_rot;
static float chassis_yaw_time_ratio_rot = 300*DEG_TO_RAD;


/*** gains to be tuned ***/
static float gain_lqr_no_remote_commands 			= 1.3;			// gain of LQR output when there're no remote commands
static float gain_unstuck_from_wall 					= 1.0;			// gain to control the unstuck from wall
static float gain_overall_u	= 6;//9.765;		// overall gain of balancing robot chassis
static float gain_ref_theta_pos_dot_com				= 0.3;			// gain that penalizes high errors in the COM linear velocity by inclining more the angular position (to both brake and accelerate)
static float gain_rot 												= 1;				// gain to rotate chassis
static float gain_lin_vel 										= 1;				// gain to individually control wheels linear velocity (not very useful)


/*** flags ***/
static int is_first_iter 							= 1;		// true if the current algorithm iteration is the first one
static int is_first_cmd_arrived				= 0;		// false until the first command from the remote controller or keyboard arrives, then it remains always true
static int should_rotate_chassis 			= 0;		// true if chassis has to rotate contiguously (defense mode)
static int should_unstuck_from_wall 	= 0;		// true if the balancing robot has to unstuck from a wall
static int is_90deg_mode_active 			= 0;		// true if balancing robot is in 90 degree mode (i.e. chassis mantains an angle of 90 degrees w.r.t. the gimbal)
static int is_brake_mode_active 			= 0;		// true if balancing robot has to brake its linear velocity (automatically, without any command from the remote controller)
static int is_chassis_rot_first_iter	= 1;		// true if we still have to do the first iteration with rotating chassis since the last time that it was rotating


/* system identification */
#if SYSID_BR_CHASSIS
struct data_sample br_chassis_theta;
struct data_sample br_chassis_theta_dot;
struct data_sample br_chassis_pos_dot;
struct data_sample br_chassis_u;
#endif

#define IS_EKF_DATA_ACQUISITION_ENABLED 1

#if IS_EKF_DATA_ACQUISITION_ENABLED
struct data_sample br_chassis_omega_dot_left;
struct data_sample br_chassis_omega_dot_right;
struct data_sample br_chassis_s;
struct data_sample br_chassis_pitch;
struct data_sample br_chassis_yaw;
struct data_sample br_chassis_s_dot;
struct data_sample br_chassis_pitch_dot;
struct data_sample br_chassis_yaw_dot;
struct data_sample br_chassis_s_hat;
struct data_sample br_chassis_pitch_hat;
struct data_sample br_chassis_yaw_hat;
struct data_sample br_chassis_s_dot_hat;
struct data_sample br_chassis_pitch_dot_hat;
struct data_sample br_chassis_yaw_dot_hat;
static float delay_data_acquisition = 15;  // [s]
#endif


/* Extended Kalman Filter */
// EKF struct and its dimensions
static ekf ekf_chassis;
static uint8_t ekf_n = EKF_n;
static uint8_t ekf_m = EKF_m;
static uint8_t ekf_q = EKF_q;
// EKF flags
uint8_t use_linear_model = TRUE;
uint8_t use_euler_method_for_state_update = FALSE;
// Fixed matrices
static float P_pred[EKF_nn];
static float S[EKF_qq];
static float K_ekf[EKF_nq];
static float I[EKF_nn];
static float F_trans[EKF_nn];
static float H_trans[EKF_qn];
static float S_inv[EKF_qq];
static float H_P_pred[EKF_qn];
static float P_pred_H_trans[EKF_nq];
// Fixed vectors
static float x_hat[EKF_n];
static float x_pred[EKF_n];						  
static float dy[EKF_q];
static float F_x_hat[EKF_n];
// EKF model matrices
static const float F[] =   {1.0000,    0.0001,    0,    0.0040,    0.0000,        0,  // Jacobian of state update function w.r.t. state
								 0,    1.0007,    0,         0,    0.0040,        0,
								 0,         0,    1,         0,         0,   0.0040,
								 0,    0.0290,    0,    1.0000,    0.0001,        0,
								 0,    0.3413,    0,         0,    1.0007,        0,
								 0,         0,    0,         0,         0,   0.9991};
//						 {1.0000,   0.0001,         0,    0.0040,    0.0000,         0,
//							   0,    1.0009,         0,    0.0000,    0.0040,         0,
//							   0,         0,    1.0000,         0,         0,    0.0040,
//							   0,    0.0330,         0,    0.9989,    0.0000,         0,
//							   0,    0.4464,         0,   -0.0100,    1.0000,         0,
//							   0,         0,         0,         0,         0,    0.9991};
								 
static const float G[] =   {0.0001e-3, 0.0001e-3,  // Jacobian of state update function w.r.t. input
							0.0006e-3, 0.0006e-3,
							0,         0,
							0.0454e-3, 0.0454e-3,
							0.3223e-3, 0.3223e-3,
							-0.1761-3, 0.1761-3};
//						   {0, 0,
//							0, 0,
//							0, 0,
//							0.0112, 0.0112,
//							-0.0988, -0.0988,
//							-0.044, 0.044};
static const float H[] =   {1, 0, 0, 0, 0, 0,  // Jacobian of measurement function w.r.t. state
							0, 1, 0, 0, 0, 0,
							0, 0, 1, 0, 0, 0,
							0, 0, 0, 1, 0, 0,
							0, 0, 0, 0, 1, 0,
							0, 0, 0, 0, 0, 1};
// EKF hyperparameters (to be tuned)
static float P[] = {0, 0, 0, 0, 0, 0,  // Initial state covariance matrix
					0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0};
static float Q[] = {1.60e-07, 0, 0, 0, 0, 0,  // Process noise covariance matrix
					0, 2.44e-07, 0, 0, 0, 0,
					0, 0, 5.81e-08, 0, 0, 0,
					0, 0, 0, 5.14e-04, 0, 0,
					0, 0, 0, 0, 1.30e-02, 0,
					0, 0, 0, 0, 0, 5.38e-04};
static float R[] = {6.25e-08, 0, 0, 0, 0, 0,  // Measurement noise covariance matrix
					0, 1.31e-06, 0, 0, 0, 0,
					0, 0, 7.79e-07, 0, 0, 0,
					0, 0, 0, 7.13e-07, 0, 0,
					0, 0, 0, 0, 3.20e-05, 0,
					0, 0, 0, 0, 0, 2.86e-05};
// Plant inputs and outputs to be used as EKF input
static float ekf_u[EKF_m];
static float ekf_y[EKF_q];
							

static const float Iw = 0.004;  // Inertia of each wheel around its rotation axis [kg*m^2]
static uint32_t ekf_start_time_ms;
static uint32_t ekf_end_time_ms;
static float ekf_elapsed_time;

/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void br_chassis_control_loop()
{

	if (is_first_iter)
	{	
		/* initialize the balancing robot chassis data structure */
		robot_init(&br_chassis, n, m);
		
		/* initialize the controllers */
		
		// LQR that, given angular position and velocity, computes the reference position of the wheels
		LQR_init(&lqr, K_lqr, Ki_lqr, m, n);
		// PID that computes a (temporary) reference angular position in order to break the unwanted linear velocities
		PID_init_(&pid_break_lin_mov, Kp_break_lin_mov, Ki_break_lin_mov, Kd_break_lin_mov, dt_chassis);
		// PID used for chassis-follow-gimbal
		PID_init_(&pid_rot, Kp_rot, Ki_rot, Kd_rot, dt_chassis);
		// PID used to independently control the linear velocity of each wheel
		PID_init_(&pid_lin_vel, Kp_lin_vel, Ki_lin_vel, Kd_lin_vel, dt_chassis);
		// PID used for the unstuck from wall
		PID_init_(&pid_unstuck_wall, Kp_unstuck_wall, Ki_unstuck_wall, Kd_unstuck_wall, dt_chassis);
		// PID that computes the theta reference when there are commands from the remote controller (to move forward/backward)
		PID_init_(&pid_ref_theta_rc, Kp_ref_theta_rc, Ki_ref_theta_rc, Kd_ref_theta_rc, dt_chassis);
		// PID of chassis wheels closed-loop control
		PID_init_(&pid_wheel, Kp_wheel, Ki_wheel, Kd_wheel, dt_chassis);
		
		
		/* initialize the filters */
		for (int wheel = 0; wheel < NUM_BR_WHEELS; wheel++) {
			
			// filter for wheels motors
			fir_init(&fir_pid_D_wheel[wheel], fir_pid_D_wheel_len, fir_pid_D_wheel_coeff, TRUE);
		}
		
		
		// initialize system identification data structures
#if SYSID_BR_CHASSIS
		data_sampling_init(&br_chassis_theta, 			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_theta_dot, 	SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_pos_dot, 		SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_u, 					SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
#endif
		
#if IS_EKF_DATA_ACQUISITION_ENABLED
		data_sampling_init(&br_chassis_omega_dot_left,	SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_omega_dot_right,	SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_pitch,			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_s,				SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_pitch,			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_yaw,				SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_s_dot,			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_pitch_dot,		SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_yaw_dot,			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_s_hat,			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_pitch_hat,		SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_yaw_hat,			SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_s_dot_hat,   	SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_pitch_dot_hat,	SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
		data_sampling_init(&br_chassis_yaw_dot_hat,		SYSID_ACQUIRED_NUM_SAMPLES, SYSID_ACQUIRE_SAMPLES_PERIOD);
#endif
	}
	

	
//	/* take relative Yaw position of chassis w.r.t. gimbal */
//	br_chassis.x[6] = ((float) (((int) ((get_yaw_gimbal_motor_measures()->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET) * 180/pi)) % 360) * pi/180);
//	/* NOTE: the "... % 360" is used to avoid doing more than 360 deg to align chassis with gimbal */
//	
//	// trasformation of angle range from 0/+360 to -180/+180 deg
//	if (br_chassis.x[6] > pi)
//		br_chassis.x[6] -= 2*pi;
//	else if (br_chassis.x[6] < -pi)
//		br_chassis.x[6] += 2*pi;
//	/* NOTE: the trasformation from 0/+360 to -180/+180 guarantees that,
//			in order to realign towards the gimbal, the chassis takes the shortest path (clockwise or counter-clockwise) */
	
	/* take relative yaw position of chassis w.r.t. gimbal */
	br_chassis.x[6] = (float) (get_yaw_gimbal_motor_measures()->contig_ang_pos_rad - CHASSIS_YAW_ENCODER_ZERO_OFFSET);
	
	// bring the chassis-gimbal relative yaw position in range [-pi,+pi]
	//br_chassis.x[6] = nearest_contig_ref_angle(br_chassis.x[6], 0);
	
	
	// check if we need to rotate chassis to 90 deg
	if (switch_is_mid(rc_left_switch) || is_key_pressed(KEY_CTRL))
		is_90deg_mode_active = TRUE;
	else
		is_90deg_mode_active = FALSE;
	
	// check if we should rotate contiguously the chassis
	if (switch_is_mid(rc_right_switch) && is_key_pressed(KEY_SHIFT) && !is_any_WASD_key_pressed())
		should_rotate_chassis = TRUE;
	else
		should_rotate_chassis = FALSE;
		
	
//	if (robot_control_mode == REMOTE_CONTROLLER_MODE)
//		is_90deg_mode_active = switch_is_mid(rc_left_switch);
//	else
//		is_90deg_mode_active = 0;
	
	
	// collect data about current robot's state from encoder and IMU
	br_chassis.x[0] = (float)((get_chassis_motor_measures(0)->contig_ang_pos_rad) * wheels_radius);
	br_chassis.x[1] = (float)((get_chassis_motor_measures(0)->ang_vel_radsec - gx) * wheels_radius);
	br_chassis.x[2] = (float)((- get_chassis_motor_measures(1)->contig_ang_pos_rad) * wheels_radius);
	br_chassis.x[3] = (float)((- get_chassis_motor_measures(1)->ang_vel_radsec - gx) * wheels_radius);
	br_chassis.x[4] = (3.1415/180)*ins_correct_angle[0];
	br_chassis.x[5] = gx;
	br_chassis.x[7] = (br_chassis.x[0] + br_chassis.x[2]) / 2;
	br_chassis.x[8] = (br_chassis.x[1] + br_chassis.x[3]) / 2;
	br_chassis.x[9] = (3.1415/180)*ins_correct_angle[2];
	br_chassis.x[10] = gz;
	
	
	// Initialize the Kalman Filter
	if (is_first_iter) {
		
		x_hat[0] = br_chassis.x[7];
		x_hat[1] = br_chassis.x[4];
		x_hat[2] = br_chassis.x[9];
		x_hat[3] = br_chassis.x[8];
		x_hat[4] = br_chassis.x[5];
		x_hat[5] = br_chassis.x[10];
		
		// TODO delete
//		ekf_init(&ekf_chassis, n_states, n_inputs, n_outputs, x_hat_init, P_init, Q, R, F, G, H,
//				 (ekf_state_eq) NULL, (ekf_output_eq) NULL, (ekf_deriv_state_eq) NULL, (ekf_deriv_state_eq) NULL,
//				 (ekf_deriv_output_eq) NULL, use_linear_model, use_euler_method_for_state_update);
//		ekf_init(&ekf_chassis, ekf_n, ekf_m, ekf_q, x_hat, P, Q, R, F, G, H,
//			  P_pred, S, K_ekf, I, x_pred, dy,
//			  F_trans, H_trans, S_inv, tmp_n1, tmp_qn, tmp_nq,
//			  (ekf_state_eq) NULL, (ekf_output_eq) NULL, (ekf_deriv_state_eq) NULL, (ekf_deriv_state_eq) NULL,
//			  (ekf_deriv_output_eq) NULL, use_linear_model, use_euler_method_for_state_update);
		
		ekf_init(&ekf_chassis, ekf_n, ekf_m, ekf_q, use_linear_model, use_euler_method_for_state_update,
			  x_hat, P, Q, R, F, G, H, P_pred, S, K_ekf, I, x_pred, dy, F_trans, H_trans, S_inv, F_x_hat, H_P_pred, P_pred_H_trans,
			  (ekf_state_eq) NULL, (ekf_output_eq) NULL, (ekf_deriv_state_eq) NULL, (ekf_deriv_state_eq) NULL, (ekf_deriv_output_eq) NULL);
	}
	
	// Update the Kalman Filter
	ekf_y[0] = br_chassis.x[7];
	ekf_y[1] = br_chassis.x[4];
	ekf_y[2] = br_chassis.x[9];
	ekf_y[3] = br_chassis.x[8];
	ekf_y[4] = br_chassis.x[5];
	ekf_y[5] = br_chassis.x[10];
	for (int i = 0; i < m; i++) {
		float torque_constant = 0.3;  // [N*m/A]
		float current = br_chassis.u[i] * M3508_C620_DAC;
		float torque = current * torque_constant;
		float omega_dot = torque / Iw;
		ekf_u[i] = omega_dot;
	}
	
	ekf_start_time_ms = HAL_GetTick();
	ekf_update(&ekf_chassis, (const float *) ekf_u, (const float *) ekf_y);
	ekf_end_time_ms = HAL_GetTick();
	ekf_elapsed_time = SEC(ekf_end_time_ms - ekf_start_time_ms);
	
	
#if IS_EKF_DATA_ACQUISITION_ENABLED
	if (SEC(HAL_GetTick()) >= delay_data_acquisition) {
		acquire_data_sample(&br_chassis_omega_dot_left,  ekf_u[0]);
		acquire_data_sample(&br_chassis_omega_dot_right, ekf_u[1]);
		acquire_data_sample(&br_chassis_s, 				 ekf_y[0]);
		acquire_data_sample(&br_chassis_pitch, 			 ekf_y[1]);
		acquire_data_sample(&br_chassis_yaw, 			 ekf_y[2]);
		acquire_data_sample(&br_chassis_s_dot, 			 ekf_y[3]);
		acquire_data_sample(&br_chassis_pitch_dot, 		 ekf_y[4]);
		acquire_data_sample(&br_chassis_yaw_dot,		 ekf_y[5]);
		acquire_data_sample(&br_chassis_s_hat, 			 ekf_chassis.x_hat.pData[0]);
		acquire_data_sample(&br_chassis_pitch_hat, 		 ekf_chassis.x_hat.pData[1]);
		acquire_data_sample(&br_chassis_yaw_hat, 		 ekf_chassis.x_hat.pData[2]);
		acquire_data_sample(&br_chassis_s_dot_hat, 		 ekf_chassis.x_hat.pData[3]);
		acquire_data_sample(&br_chassis_pitch_dot_hat,	 ekf_chassis.x_hat.pData[4]);
		acquire_data_sample(&br_chassis_yaw_dot_hat,	 ekf_chassis.x_hat.pData[5]);
	}
#endif
	
	
	
	if (is_first_iter) {
		br_chassis.ref[0] = br_chassis.x[0];
		br_chassis.ref[2] = br_chassis.x[2];
	}

	// collect commands sent by means the remote controller
	if (switch_is_up(rc_right_switch)) {
		
		cmd_fwd_bwd = (float) rc_left_joystick_ud; 			// left stick, up/down movement
		cmd_right_left = (float) rc_left_joystick_rl;		// left stick, right/left movement
	}
	else if (switch_is_mid(rc_right_switch)) {
		
		// get WASD commands from keyboard
		compute_weights_WASD_keys(dt_chassis);
		
		/* convert WASD commands to chassis commands */
		cmd_fwd_bwd			 = MAX_RC_TILT * weight_fwd_key;
		cmd_fwd_bwd			-= MAX_RC_TILT * weight_bwd_key;
		cmd_right_left	 = MAX_RC_TILT * weight_right_key;
		cmd_right_left	-= MAX_RC_TILT * weight_left_key;
		
		// saturate, just to be sure
		saturate(&cmd_fwd_bwd, MAX_RC_TILT);
		saturate(&cmd_right_left, MAX_RC_TILT);
		
		// if SHIFT key is pressed, go slower
		if (is_key_pressed(KEY_SHIFT)) {
			
			cmd_fwd_bwd *= 0.5;
			cmd_right_left *= 0.5;
		}
	}
	
	
	
	/*** unstuck from wall ***/
	
	// if this is first algorithms iteration, start the routine to detect if the robot is stuck on wall
	if (is_first_iter) {
		clock_tick_ms_before_unstuck_wall = HAL_GetTick();
		chassis_accel_double_integr = 0;
		theta_before_unstuck_wall = br_chassis.x[4];
		theta_com_before_unstuck_wall = theta_com;
	}
	
	/* check if we should unstuck from wall */
	if (!should_unstuck_from_wall) {
		
		// conditions to be satisfied (all together, for a certain time interval) in order to trigger the "unstuck from wall" routine
		if ((	fabs(br_chassis.u[0]) >= u_trigger_unstuck_wall &&
					fabs(br_chassis.u[1]) >= u_trigger_unstuck_wall &&
					sign_zero_undef(br_chassis.u[0]) != sign_zero_undef(br_chassis.u[1]) &&
					fabs(br_chassis.x[4] - theta_before_unstuck_wall) <= theta_tol_unstuck_wall &&
					fabs(chassis_accel_double_integr) < chassis_accel_double_integr_tol)
					|| (is_key_raising_edge(KEY_B))) {
			
			if ((HAL_GetTick() - clock_tick_ms_before_unstuck_wall >= time_before_unstuck_wall*1e3)
					|| (is_key_raising_edge(KEY_B))) {
				
				// take info regarding the "unstuck from wall" routine
				clock_tick_ms_while_unstuck_wall = HAL_GetTick();
				sign_ref_pos_dot_unstuck_wall = - sign_zero_undef(br_chassis.u[0]);
				for (int i = 0; i < 2*NUM_BR_WHEELS; i++)
					br_chassis.integr_error[i] = 0;
				
				// trigger the "unstuck from wall" routine
				should_unstuck_from_wall = 1;
			}
			else {
				
				// continue to compute the double integral of microprocessor's acceleration
				chassis_accel_double_integr += (1/2)*(ay*cos(br_chassis.x[4]))*dt_chassis*dt_chassis;
				
				// sample the theta COM value before the unstuck from wall
				theta_com_before_unstuck_wall = theta_com;
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
		br_chassis.integr_error[0] = 0;
		br_chassis.integr_error[2] = 0;
		
		// reset the theta COM (to avoid wrong values computed during the "unstuck from wall" routine)
		theta_com = theta_com_before_unstuck_wall;
	}
	
	
	/* disable the not working or not yet implemented functionalities */
#if !IS_90DEG_MODE_ENABLED
	is_90deg_mode_active = 0;
#endif
	
#if !IS_CHASSIS_ROT_MODE_ENABLED
	should_rotate_chassis = 0;
#endif
	
#if !IS_UNSTUCK_FROM_WALL_MODE_ENABLED
	should_unstuck_from_wall = 0;
#endif
	
	float ref_chassis_yaw;		// reference for chassis-gimbal yaw relative angle, to be reached by either the chassis' FORWARD or BACKWARD part
	float mag_cmd_rc;					// magnitude of the command from the remote controller
	
	mag_cmd_rc = sqrt(pow(cmd_fwd_bwd, 2) + pow(cmd_right_left, 2));
	
	ref_chassis_yaw = 0;
	if (switch_is_up(rc_left_switch) && mag_cmd_rc != 0)
		ref_chassis_yaw += atan2(cmd_right_left/mag_cmd_rc, cmd_fwd_bwd/mag_cmd_rc);
	
	/* set reference for chassis orientation (chassis-follow-gimbal, 90 degree mode or rotating chassis mode) */
	if (is_90deg_mode_active) {
		
		float ref_chassis_90deg_right;		// reference for chassis-gimbal yaw relative angle to reach the "90 degree" mode with chassis' FORWARD part at the RIGHT of the chassis
		float ref_chassis_90deg_left;			// reference for chassis-gimbal yaw relative angle to reach the "90 degree" mode with chassis' FORWARD part at the LEFT of the chassis
		
		ref_chassis_90deg_right = nearest_contig_ref_angle(pi/2, br_chassis.x[6]);
		ref_chassis_90deg_left = nearest_contig_ref_angle(-pi/2, br_chassis.x[6]);
		
		if (fabs(br_chassis.x[6] - ref_chassis_90deg_right) <= pi/2)
			br_chassis.ref[6] = ref_chassis_90deg_right;
		else
			br_chassis.ref[6] = ref_chassis_90deg_left;
	}
	else if (should_rotate_chassis) {
		
		if (is_chassis_rot_first_iter) {
			
			time_ms_begin_rot = HAL_GetTick();
			chassis_yaw_begin_rot = br_chassis.x[6];
			gimbal_yaw_begin_rot = gimbal.x[0];
			is_chassis_rot_first_iter = FALSE;				// NOTE: this flag gets set again to 1 at the end of the this code (if some criteria are fulfilled)
		}
		
		br_chassis.ref[6] = chassis_yaw_begin_rot + chassis_yaw_time_ratio_rot * ((HAL_GetTick() - time_ms_begin_rot)*1e-3);
		br_chassis.ref[6] += (gimbal.x[0] - gimbal_yaw_begin_rot);
	}
	else {
		
		if (switch_is_up(rc_left_switch) && mag_cmd_rc == 0) {
			
			br_chassis.ref[6] = br_chassis.x[6];
		}
		else {
			
			float ref_chassis_yaw_aligned = ref_yaw_chassis_rot(ref_chassis_yaw, br_chassis.x[6]);

			br_chassis.ref[6] = nearest_contig_ref_angle(ref_chassis_yaw_aligned, br_chassis.x[6]);
		}
	}
	
	
	
	/* define and acquire reference signals (depending on the current situation) */
	float ref_theta_break_lin_mov;
	float *ref_theta_com;

	if (cmd_fwd_bwd == 0 && cmd_right_left == 0) {
		
		// initial references values
		br_chassis.ref[1] = 0;
		br_chassis.ref[3] = 0;
		br_chassis.ref[4] = 0;
		br_chassis.ref[5] = 0;
		br_chassis.ref[8] = 0;
		
		// compute COM linear velocity error
		ref_error_state(&br_chassis, 8, dt_chassis);
		reset_integr_error_if_overshoot(&br_chassis, 8);
		
		/* adapt balance point to COM shift */
		if (!is_brake_mode_active)
			ref_theta_com = &theta_com;
		else
			ref_theta_com = &theta_com_brake;
		
		// update the COM equilibrium point
		*ref_theta_com += dt_chassis * br_chassis.x[8] * gain_theta_com;
		saturate(ref_theta_com, sat_theta_com);
		
		// compute the "temporary" theta reference to break the unwanted linear movements
		ref_theta_break_lin_mov = - PID_control(&pid_break_lin_mov,
																						br_chassis.error[7],			br_chassis.integr_error[7], br_chassis.error[8],
																						sat_pid_P_break_lin_mov,	0.0,												sat_pid_D_break_lin_mov,
																						NULL,											NULL,												NULL);
		
		/* activate braking mode to brake COM linear velocity (if it is going too fast) */
		if (is_first_iter)
			pos_com_before_brake = br_chassis.x[7];
		
		// check if we have to trigger the brake mode
		if (!is_brake_mode_active) {
			
			if (fabs(br_chassis.x[8]) >= brake_trigger_com_lin_vel && sign_zero_undef(br_chassis.x[1]) == sign_zero_undef(br_chassis.x[3])) {
				
				if ((fabs(br_chassis.x[7] - pos_com_before_brake) >= brake_trigger_lin_pos) && is_first_cmd_arrived &&
					(HAL_GetTick() - clock_tick_ms_last_brake >= min_time_between_brakes*1e3 || HAL_GetTick() - clock_tick_ms_last_brake <= max_time_in_brakes*1e3)) {
					/* save info regarding the brake mode */
					sign_com_lin_vel = sign_zero_undef(br_chassis.x[8]);		// store sign of COM linear velocity just before the trigger of braking mode								
					theta_com_brake = theta_com;														// store COM equilibrium angle just before the trigger of braking mode
					
					// update the time of the last (i.e., most recent) brake
					clock_tick_ms_last_brake = HAL_GetTick();
					
					// activate the brake mode
					is_brake_mode_active = 1;																// trigger braking mode
				}
			}
			else {		// don't trigger the brake mode
				
				pos_com_before_brake = br_chassis.x[7];
				theta_com_before_brake = theta_com;
			}
		}
		
		
		/* deactivate braking mode (if we managed to brake the COM linear velocity) */
		if (is_brake_mode_active && (sign_zero_undef(br_chassis.x[8]) != sign_com_lin_vel || fabs(br_chassis.x[8]) < tol_lin_vel_com_exit_brake)) {
			
			// restore the normal balance configuration (for post-brake mode)
			theta_com = theta_com_before_brake;							// resume COM equilibrium angle that there was before braking mode
			saturate_in_range(&theta_com, -9*pi/180, 9*pi/180);
			pos_com_before_brake = br_chassis.x[7];
			
			// disable brake mode
			is_brake_mode_active = 0;
			
			// NOTE: when the brake mode gets disabled, theta_com can be set either to 0 or theta_com_before_brake
		}
		
		
		// theta
		br_chassis.ref[4] += *ref_theta_com;
		br_chassis.ref[4] += ref_theta_break_lin_mov;
		
		float sat_theta_lb = theta_com - sat_ref_theta_physical_limit;
		float sat_theta_ub = theta_com + sat_ref_theta_physical_limit;
		
		saturate_in_range(&br_chassis.ref[4], sat_theta_lb, sat_theta_ub);
		saturate(&br_chassis.ref[4], sat_ref_theta_physical_limit);
		
		
		// pos_dot of wheels
		br_chassis.ref[1] = br_chassis.x[1];
		br_chassis.ref[3] = br_chassis.x[3];
	
	}
	else {		// there are commands from keyboard/remote controller
		
		is_first_cmd_arrived = 1;			// at least a command from keyboard/remote controller arrived
		
		pos_com_before_brake = br_chassis.x[7];
		theta_com_before_brake = theta_com;
		
		/*** definition of variables ***/
		float cmd_rc;										// command received from remote controller (in range [-660, +660]
		float ref_theta_pos_dot_com;		// component of theta reference (starting at the COM) that depends on the COM linear velocity error w.r.t. the reference
		float sat_ref_theta;						// saturation for theta reference
		
		/*** reference of theta dot ***/
		br_chassis.ref[5] = 0;
		
		
		/*** reference of theta ***/
		
		// get COM linear velocity reference from keyboard/remote controller
		if (switch_is_up(rc_left_switch))
			cmd_rc = sqrt(pow(cmd_fwd_bwd, 2) + pow(cmd_right_left, 2));
		else
			cmd_rc = (!is_90deg_mode_active) ? cmd_fwd_bwd : cmd_right_left;
		
		// account for the current chassis' orientation
		if (!is_90deg_mode_active || (switch_is_up(rc_left_switch) && is_key_pressed(KEY_CTRL)))
			cmd_rc *= ref_yaw_chassis_dir(ref_chassis_yaw, br_chassis.x[6]);
		else
			cmd_rc *= ref_yaw_chassis_dir(ref_chassis_yaw, br_chassis.x[6] - pi/2);
		
		// reference of COM linear velocity (corresponding to the keyboard/remote controller command)
		br_chassis.ref[8] = (cmd_rc / MAX_RC_TILT) * (max_ref_pos_dot_rc);
		
		
		// compute COM linear velocity error
		ref_error_state(&br_chassis, 8, dt_chassis);
		
		// account for situations where remote controller commands are being sent when robot is in brake mode
		if (is_brake_mode_active) {
			theta_com = theta_com_before_brake;
			saturate_in_range(&theta_com, -1*pi/180, 9*pi/180);
			
			// disable brake mode
			is_brake_mode_active = 0;
		}
		
		// compute theta reference
		br_chassis.ref[4] = theta_com;			// make theta reference start at the COM equilibrium point
		
		ref_theta_pos_dot_com = PID_control(&pid_ref_theta_rc,
																				br_chassis.error[8],	br_chassis.integr_error[8],	0 - ay,			// compute theta reference (from COM equilibrium point) with a PID
																				UNDEF,														UNDEF,					UNDEF,
																				NULL,															NULL,						NULL);
		ref_theta_pos_dot_com *= (1 + fabs(br_chassis.x[8]) * gain_ref_theta_pos_dot_com);		// penalize high COM linear velocities by inclining more theta reference
		
		// saturate theta reference
		sat_ref_theta = range_lin_prop(fabs(br_chassis.x[8]), pos_dot_com_sat_ref_theta_bounds[0], pos_dot_com_sat_ref_theta_bounds[1], sat_ref_theta_cmd[0], sat_ref_theta_cmd[1]);
		saturate(&ref_theta_pos_dot_com, sat_ref_theta);								// saturation from COM (which value depends on the COM linear velocity)
		br_chassis.ref[4] -= ref_theta_pos_dot_com;
		saturate(&br_chassis.ref[4], sat_ref_theta_physical_limit);			// saturation from 0 angle of board (due to physical limitations to avoid to touch the ground)
	}
	
	
	/* set wheels linear velocity reference, based on whether we are in "unstuck from wall" routine or not */
	if (should_unstuck_from_wall) {
		
		br_chassis.ref[1] = ref_pos_dot_unstuck_wall * sign_ref_pos_dot_unstuck_wall;
		br_chassis.ref[3] = ref_pos_dot_unstuck_wall * sign_ref_pos_dot_unstuck_wall;
	}
	else {
		
		br_chassis.ref[1] = br_chassis.x[1];
		br_chassis.ref[3] = br_chassis.x[3];
	}
	
	
	
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
		saturate(&br_chassis.u[wheel], MAX_BR_POS_LQR_THETA_OUT);
	}

	
	/* closed-loop PID for left and right wheel */
	{
		int state = 0;
		int output = 0;
		
		// for each wheel...
		while (state == 0 || state == 2) {
			
			/* definition of variables */
			float ref_offset_pos;								// PID control of a wheel
			float ref_pos_brake;

			ref_offset_pos = br_chassis.u[output] * max_lin_pos;
			
			if (0/*is_brake_mode_active*/) {
				ref_pos_brake = - br_chassis.x[state+1] * gain_pos_brake * (1 + gain_pos_brake_theta * fabs(br_chassis.x[4] - theta_com));
				ref_offset_pos += sign_zero_undef(ref_pos_brake) * max(fabs(ref_pos_brake), min_ref_pos_brake);
			}
			
			br_chassis.ref[state] = br_chassis.x[state] + ref_offset_pos;
			ref_error_state(&br_chassis, state, dt_chassis);
			saturate(&br_chassis.integr_error[state], sat_integr_error_pos);
			
			br_chassis.u[output] = PID_control(&pid_wheel,
																				br_chassis.error[state],	br_chassis.integr_error[state], 0 - br_chassis.x[state+1],
																				UNDEF,										UNDEF,													UNDEF,
																				NULL,											NULL,														&fir_pid_D_wheel[output]);
			
			// update counters
			state += 2;
			output += 1;
		}
	}
	

	/* rotate chassis (to follow gimbal or to go 90 degrees mode) */
	float u_rot_chassis = gain_rot * PID_control(&pid_rot,
																							br_chassis.error[6],	br_chassis.integr_error[6], br_chassis.deriv_error[6],
																							UNDEF,								UNDEF,											UNDEF,
																							NULL,									NULL,												NULL);
	saturate(&u_rot_chassis, sat_pid_rot);
	
	/* control wheels linear velocity */
//	float u_pos_dot_L = gain_lin_vel * PID_control(&pid_lin_vel,
//																								br_chassis.error[1],	br_chassis.integr_error[1], br_chassis.deriv_error[1],
//																								UNDEF,								UNDEF,											UNDEF,
//																								NULL,									NULL,												NULL);
//	float u_pos_dot_R = gain_lin_vel * PID_control(&pid_lin_vel,
//																								br_chassis.error[3],	br_chassis.integr_error[3], br_chassis.deriv_error[3],
//																								UNDEF,								UNDEF,											UNDEF,
//																								NULL,									NULL,												NULL);
	
	
	
	
	
	
	
	
	
	
	/* control wheels linear velocity */
	float u_unstuck_wall_L = gain_unstuck_from_wall * PID_control(&pid_unstuck_wall,
																																br_chassis.error[1],	br_chassis.integr_error[1], br_chassis.deriv_error[1],
																																UNDEF,								UNDEF,											UNDEF,
																																NULL,									NULL,												NULL);
	float u_unstuck_wall_R = gain_unstuck_from_wall * PID_control(&pid_unstuck_wall,
																																br_chassis.error[3],	br_chassis.integr_error[3], br_chassis.deriv_error[3],
																																UNDEF,								UNDEF,											UNDEF,
																																NULL,									NULL,												NULL);
	
	
	// deny any chassis command when the chassis is rotating contiguously
//	if (should_rotate_chassis) {
//		
//		br_chassis.u[0] = 0.0;
//		br_chassis.u[1] = 0.0;
//	}
		
	
	/* sum up control signals of individual tasks to the overall control signal */
	if (!should_unstuck_from_wall) {
	
		br_chassis.u[0] += u_rot_chassis;
		br_chassis.u[1] -= u_rot_chassis;
	}
	else {
		
		if (HAL_GetTick() - clock_tick_ms_while_unstuck_wall < time_shut_down_unstuck_wall*1e3) {
		
			br_chassis.u[0] = 0;
			br_chassis.u[1] = 0;
		}
		else {
			
			br_chassis.u[0] = u_unstuck_wall_L;
			br_chassis.u[1] = u_unstuck_wall_R;
		}
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
	
	// if needed, reset the flag of first chassis rotation iteration (for next iterations)
	if (!should_rotate_chassis)
		is_chassis_rot_first_iter = TRUE;
	
	// first algorithm iteration terminates here
	is_first_iter = 0;
}



