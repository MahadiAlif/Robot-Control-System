#ifndef CONTROL_UTIL_H
#define CONTROL_UTIL_H

#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include "gimbal_control.h"
#include "math_util.h"
#include "br_chassis_control.h"
#include "shoot_rev_control.h"
#include "std_chassis_control.h"
#include <struct_typedef.h>

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/

/* macros to remain fixed */
// remote controller
#define MAX_RC_TILT										660.0f				// value corresponding to the max inclination of remote controller sticks (along every direction)
// motors
#define M3508_C620_ADC								819.2f				// analog-to-digital convertion ratio of M3508 motor + C620 speed controller (819.2 = 16384/20A)
#define M3508_C620_DAC								1.220703e-3		// digital-to-analog convertion ratio of M3508 motor + C620 speed controller (1.220703e-3 = 1/819.2 = 20A/16384)
#define GM6020_ADC										1250.0f				// analog-to-digital convertion ratio of GM6020 motor (1250 = 30000/24V)
#define GM6020_DAC										8e-4					// digital-to-analog convertion ratio of GM6020 motor (8e-4 = 1/1250 = 24V/30000)
#define GM6020_DISCRETE_TORQUE_RATIO	1811.943			// ratio between the discrete input of GM6020 motor and its output torque (NOTE: the ratio was computed experimentally)
// computer vision
#define CV_PITCH_FIT_COEFFS	6							// number of coefficients used to fit the experimentally-collected data for CV-driven pitch control. The coefficients are: bias, theta, v0, theta^2, theta*v0, v0^2 (in this order)
// sentry's state machine
#define IDLE					0
#define MOVE_TO_SPOT	1
#define PATROL				2
#define ENEMY_LOCK		3
#define ENEMY_WARN		4
#define ENEMY_ENGAGE	5
// game progress
#define PRE_COMPETITION_STAGE			0
#define SETUP_PERIOD							1
#define INITIALIZATION_STAGE			2
#define FIVE_SECOND_COUNTDOWN 		3
#define IN_COMBAT									4
#define CALC_COMPETITION_RESULTS	5


/* macros to be manually tuned */
#define MAX_ROBOT_STATES	12						// max number of states that each robot can have (arbitrary)
#define MAX_ROBOT_INPUTS	6						// max number of inputs that each robot can have (arbitrary)
#define MAX_SAMPLED_DATA 	1100					// max number of data that can be sampled and stored in a single array (arbitrary)
#define MAX_FIR_FILTER_LEN	500						// max buffer length of a FIR (Finite Impulse Response) low-pass filter


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/* sentry's state machine */
extern uint8_t sentry_state;										// current sentry's state (received from ROS), used to decide how the sentry should behave
extern uint8_t sentry_state_prev;								// previous sentry's state
extern uint8_t is_sentry_spot_reached;					// 1 if the sentry's chassis target on ground is reached (in MOVE_TO_SPOT state)
extern uint8_t is_cv_enemy_lost;								// 1 if we have lost all enemies (i.e., the CV didn't detect any enemy) since a certain amount of time
extern uint8_t is_cv_enemy_stalled;							// 1 if the detected enemy has been staying the middle of the short/long distance range for a certain amount of time
extern float time_ms_last_enemy_detected;
extern float time_ms_enemy_began_to_stall;
extern float time_enemy_lost;										// time that should elapse to declare that we have lost all enemy robots
extern float time_enemy_stalled;								// time that should elapse to declare that the enemy robot is stalling on the short/long distance range

/* keyboard keys long press weights: values in range [0,1] that determine for how long a keyboard key has been pressed consecutively until now. */
extern float weight_fwd_key;		// weight of forward movement key 	(W)
extern float weight_left_key;		// weight of left movement key 			(A)
extern float weight_bwd_key;		// weight of backward movement key 	(S)
extern float weight_right_key;	// weight of right movement key 		(D)

/* mouse commands */
extern float mouse_speed_x;
extern float mouse_speed_y;
extern float mouse_speed_x_prev;
extern float mouse_speed_y_prev;

/*** gimbal mouse movement values ***/
extern uint32_t clock_ms_mouse_gimbal;
extern uint32_t clock_ms_mouse_gimbal_prev;
extern float mouse_meters_yaw_rad_ratio;			// if mouse travels 0.01 meters, then yaw has to travel 35*pi/180 rad
extern float mouse_meters_pitch_rad_ratio;		// if mouse travels 0.005 meters, then yaw has to travel 30*pi/180 rad

/* flags */
extern int shoot_motors_speed_increased;
extern int mouse_sensibility_pitch_increased;
extern int mouse_sensibility_yaw_increased;
extern int theta_balance_value_increased;



/************************************************************************************************************************
	NAME: tf
	
	DESCRIPTION: data structure that can contain a discrete transfer function

					b[m]*z^{m-n} + b[m-1]*z^{m-n-1} + ... + b[0]*z^{-n}
	TF = --------------------------------------------------------
			 a[n] + a[n-1]*z^{-1} + a[n-2]*z^{-2} + ... + a[0]*z^{-n}
		
	num = {b[n], b[n-1], ..., b[0]};		NOTE: here the index is intentionally "n", not "m"
	den = {a[n], a[n-1], ..., a[0]};

	y = {y[k], y[k-1], ..., y[k-n]};
	u = {u[k], u[k-1], ..., u[k-n]};

	EXAMPLES:
	1.
				5*z^2 - 9.75*z + 4.753
	 --------------------------------
	 z^3 - 2.9*z^2 + 2.803*z - 0.9029

	m = 2;
	n = 3;
	num = {0, 5, -9.75, 4.753};					NOTE: here num[0] = 0 (i.e. b[3] = 0) because m = 2, n = 3, n-m = 1 (there's a delay of one time unit)
	den = {1, -2.9, 2.803, -0.9029};
	
	NOTE: you must have "m <= n" in order to have a proper transfer function
************************************************************************************************************************/
struct tf {

    float 	dt;			// sample time of the discrete tf
    uint8_t m;			// order of tf numberator
    uint8_t n; 			// order of tf denominator
    float 	num[MAX_POLYN_ORDER + 1];  		// tf numberator coefficients
    float 	den[MAX_POLYN_ORDER + 1];    	// tf denominator coefficients
    float 	y[MAX_POLYN_ORDER + 1];      	// tf outputs
    float 	u[MAX_POLYN_ORDER + 1];      	// tf inputs
};


/************************************************************************************************************************
	NAME: data_sample
	
	DESCRIPTION: data structure that can contain sampled data (to be either sent to motors or received from sensors)
************************************************************************************************************************/
struct data_sample {
	
	float		samples[MAX_SAMPLED_DATA];	// array of sampled data
	uint32_t	num_samples;	// number of data to be sampled
	uint32_t 	k;						// counter of sampled data
	float 		dt;						// sampling period: time to elapse between two consecutive sampled values (in seconds)
	uint32_t 	clock_tick_ms_last_iter;		// milliseconds elapsed between the beginning of the code execution and the last code iteration (measured with HAL_GetTick())
	
	// NOTE: the actual elapsed time between two consecutive samples could be a little larger than "dt" (for sure not smaller)
};


/************************************************************************************************************************
	NAME: fir
	
	DESCRIPTION: data structure that implements a FIR (Finite Impulse Response) LPF (Low-Pass Filter)
************************************************************************************************************************/
struct fir {
	
	uint32_t len;															// FIR filter length (i.e. length of the coefficients and buffer arrays)
	float coeff[MAX_FIR_FILTER_LEN];					// FIR filter coefficients
	float buf[MAX_FIR_FILTER_LEN];						// circular buffer of previous FIR filter inputs (used to compute the filter's output through a weighted average, which weights are the FIR filter coefficients)
	int buf_offset;														// offset inside the circular buffer that always points to the *newest* element
	uint8_t is_feedback_from_filtered_data;		// set to 1 if the FIR filter should insert inside its buffer the filtered values, otherwise 0 (insert the raw values).
	
};


/************************************************************************************************************************
	NAME: cv_gimbal_auto_aim
	
	DESCRIPTION: data structure containing all the things needed for the gimbal yaw/pitch auto-aiming through Computer Vision (CV)
************************************************************************************************************************/
struct cv_gimbal_auto_aim {
	
	// x,y,z w.r.t. left camera of ZED stereo camera
	fp32 x_cam_left;
	fp32 y_cam_left;
	fp32 z_cam_left;
	
	// x,y,z w.r.t. left camera of ZED stereo camera in previous time instants
	fp32 x_cam_left_prev;
	fp32 y_cam_left_prev;
	fp32 z_cam_left_prev;
	
	// x,y,z w.r.t. center of ZED stereo camera
	fp32 x_cam;
	fp32 y_cam;
	fp32 z_cam;
	
	// yaw and pitch filters
	struct fir fir_yaw_cv;
	struct fir fir_pitch_cv;
	
	// buffer sizes of yaw and pitch filters
	uint8_t yaw_filter_buf_size;
	uint8_t pitch_filter_buf_size;
	
	// data-fitted coeffients to estimate the gimbal's pitch angle
	float pitch_fit_coeff[CV_PITCH_FIT_COEFFS];
	
	// distance/height of enemy's armor
	float d_armor;      // distance from enemy's armor (on ground, i.e., along Y axis)		[m]
	float h_armor;      // altitude of enemy's armor (from ground)		[m]
	float d_xyz;				// Euclidean distance to the enemy's armor
	
	// offsets due to the ZED2's physical structure
	fp32 distance_left_cam_to_center_zed2;							// distance (in absolute value) between camera sx and center of ZED2 stereocamera
	fp32 distance_center_zed2_to_center_shooting;				// vertical distance (in absolute value) between the ZED2's center and the shooter
	fp32 distance_left_cam_to_center_zed_mini;					// distance (in absolute value) between camera sx and center of ZED Mini stereocamera
	fp32 distance_center_zed_mini_to_center_shooting;		// vertical distance (in absolute value) between the ZED Mini's center and the shooter
	
	// time elapsed between consecutive CV detections
	float dt;		// [s]
	
	// initial bullet's velocity
	float v0;   // [m/s]
	
	// shooter height from ground
	float h_shooter;     // [m]
	
	// shooting frequency and range
	float min_shoot_distance;   		// [m]
	float max_shoot_distance;   		// [m]
	float max_enemy_lock_distance;	// [m]
	float max_shoot_freq;       		// [Hz]
	float shoot_freq_enemy_warn;		// [Hz]
	
	// clock timestamp of the current and previous CV detection
	uint32_t clock_ms_curr_iter;   // [ms]
	uint32_t clock_ms_prev_iter;   // [ms]

	// flag that is set to false until the CV auto-aim function doesn't get called for the first time (from that moment, it remains true)
	uint8_t first_iter_done;
	
	
	
//	// enemy's coordinates (from stereo camera)
//	float x_cam;    // [m]
//	float y_cam;    // [m]
//	float z_cam;    // [m]
//	
//	// clock timestamp of the current and previous CV detection
//	uint32_t clock_ms_curr_iter;   // [ms]
//	uint32_t clock_ms_prev_iter;   // [ms]

//	// initial bullet's velocity
//	float v0;   // [m/s]

//	// average bullet's velocity along its whole trajectory
//	float vp;   // [m/s]

//	// shooter height from ground
//	float h_shooter;     // [m]

//	// enemy's coordinates predictions
//	float x_hat;    // [m]
//	float y_hat;    // [m]
//	float z_hat;    // [m]
//	
//	// enemy's velocity predictions
//	float vx_hat;    // [m]
//	float vy_hat;    // [m]
//	float vz_hat;    // [m]

//	// enemy's coordinates predictions in the previous algorithm iteration
//	float x_hat_prev;   // [m]
//	float y_hat_prev;   // [m]
//	float z_hat_prev;   // [m]

//	// position and velocity buffer sizes
//	uint8_t pos_filter_buf_size;
//	uint8_t vel_filter_buf_size;

//	// filters for x,y,z position
//	struct fir x_filter;
//	struct fir y_filter;
//	struct fir z_filter;

//	// filters for vx,vy,vz velocities
//	struct fir vx_filter;
//	struct fir vy_filter;
//	struct fir vz_filter;

//	// data fitting coefficients for gimbal's pitch prediction
//	float pitch_fit_coeff[CV_PITCH_FIT_COEFFS];         // order of coefficients: bias, theta, v0, theta^2, theta*v0, v0^2

//	// shooting frequency and range
//	float min_shoot_distance;   // [m]
//	float max_shoot_distance;   // [m]
//	float max_shoot_freq;       // [Hz]

//	// flag that is set to false until the CV auto-aim function doesn't get called for the first time (from that moment, it remains true)
//	uint8_t first_iter_done;
};



// forward declaration of robot data structure
struct robot;

/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void saturate(float *value, float bound);


void saturate_in_range(float *value, float lb, float ub);



/************************************************************************************************************************
	NAME: wn_gauss_gen
	
	DESCRIPTION: generates a random white noise array (with mean=0, variance=1)
	
	ARGUMENTS:
	- wn_gauss:				array to be filled up
	- num_samples:		number of samples to be generated
************************************************************************************************************************/
void wn_gauss_gen(float *wn_gauss, unsigned int num_samples);


/************************************************************************************************************************
	NAME: rand_bin_gen
	
	DESCRIPTION: generates a random binary distributed array (filled with only 0 and 1)
	
	ARGUMENTS:
	- v:							array to be filled up
	- num_samples:		number of samples to be generated
************************************************************************************************************************/
void rand_bin_gen(float *v, unsigned int num_samples);


/************************************************************************************************************************
	NAME: rand_unif_gen
	
	DESCRIPTION: generates a random uniformly distributed array in range [0,1]
	
	ARGUMENTS:
	- v:							array to be filled up
	- num_samples:		number of samples to be generated
************************************************************************************************************************/
void rand_unif_gen(float *v, unsigned int num_samples);


void decide_if_mantain_gimbal_position(void);


/************************************************************************************************************************
	NAME: compute_weights_WASD_keys
	
	DESCRIPTION:
	- for W,A,S,D keys, determines for how long each of them has been pressed consecutively until now.
	- all weights are in range [0,1].
	- each weight reaches the value 1 in 1 second (starting from 0).
	
	ARGUMENTS:
	- dt:		time period (i.e. inverse of frequency) of the code that executes the function
************************************************************************************************************************/
void compute_weights_WASD_keys(float dt);


void update_mouse_position_gimbal(float sample_period, float *cmd_yaw, float *cmd_pitch);
void check_keyboard_commands_shoot_motors(uint8_t shooting_control_mode);


void shoot_model_init(shoot_motors_model_t *model);
void rev_model_init(rev_motor_model_t *model);
// TODO substitute the above functions with the following one
void robot_init(struct robot *robot, uint8_t n, uint8_t m);


/************************************************************************************************************************
	NAME: tf_init
	
	DESCRIPTION: initialize a discrete transfer function

	ARGUMENTS:
	- tf:		pointer to the tf
	- dt:   tf sample time
	- m:    order of tf numerator
	- n:		order of tf denominator
	- num:	tf numerator
	- den:	tf denominator

	EXAMPLES:
	1.
	      5*z^2 - 9.75*z + 4.753
	 --------------------------------
	 z^3 - 2.9*z^2 + 2.803*z - 0.9029

		Inputs to function:
		dt = 0.01;
		m = 2;
		n = 3;
		num = {5, -9.75, 4.753};						NOTE: not {0, 5, -9.75, 4.753}
		den = {1, -2.9, 2.803, -0.9029};
		
		Outputs of function:
		tf->dt = 0.01;
		tf->m = 2;
		tf->n = 3;
		tf->num = {0, 5, -9.75, 4.753};			NOTE: now we have {0, 5, -9.75, 4.753}, where the initial 0 takes into account the delay of "n-m = 1" time units
		tf->den = {1, -2.9, 2.803, -0.9029};
************************************************************************************************************************/
void tf_init(struct tf *tf, float dt, uint8_t m, uint8_t n, float *num, float *den);


/************************************************************************************************************************
	NAME: zpk2tf
	
	DESCRIPTION: given the zeros, poles and gain (NOT steady-state gain) of a transfer function, computes the numerator and denominator coefficients of the transfer function

	ARGUMENTS:
	- m:    tf numerator order
	- n:    tf denominator order
	- z:    array of tf zeros (can also be complex numbers)
	- p:    array of tf poles (can also be complex numbers)
	- k:   	tf gain
	- num:  tf numerator
	- den:  tf denominator

	EXAMPLES:
	1.
			 5 * (z- 0.98) * (z- 0.97)            5*z^2 - 9.75*z + 4.753
	 --------------------------------- = --------------------------------
	 (z- 0.99) * (z- 0.96) * (z- 0.95)   z^3 - 2.9*z^2 + 2.803*z - 0.9029

	if...
		m = 2;
		n = 3;
		z = {0.98, 0.97};
		p = {0.99, 0.96, 0.95};
		k = 5;
	then...
		num = {5, -9.75, 4.753};
		den = {1, -2.9, 2.803, -0.9029};
************************************************************************************************************************/
//void zpk2tf(uint8_t m, uint8_t n, float complex *z, float complex *p, float k, float *num, float *den);


/************************************************************************************************************************
	NAME: data_sampling_init
	
	DESCRIPTION: initializes the given "data_sample" data structure for sending/acquiring data samples

	ARGUMENTS:
	- ds:    					pointer to the data structure to be initialized
	- num_samples:		number of data to be sampled
	- dt:							sample period (in seconds)
************************************************************************************************************************/
void data_sampling_init(struct data_sample *ds, uint32_t num_samples, float dt);


/************************************************************************************************************************
	NAME: acquire_data_sample
	
	DESCRIPTION: stores the given sample (i.e. value) into the given "data_sample" data structure

	ARGUMENTS:
	- ds:    		pointer to the data structure where to store the sample
	- sample:		sample to be stored
************************************************************************************************************************/
void acquire_data_sample(struct data_sample *ds, float sample);


/************************************************************************************************************************
	NAME: send_data_sample
	
	DESCRIPTION: returns a data sample (from the given "data_sample" data structure) to be send to robot motors, according to the sample period

	ARGUMENTS:
	- ds:    		pointer to the data structure from where to get the sample to be sent to the motors
************************************************************************************************************************/
float send_data_sample(struct data_sample *ds);


/************************************************************************************************************************
	NAME: dcgain
	
	DESCRIPTION: returns the steady-state gain of the given transfer function
************************************************************************************************************************/
float dcgain(struct tf *tf);


/************************************************************************************************************************
	NAME: set_dcgain
	
	DESCRIPTION: modifies the numerator of the given transfer function in order to set its steady-state gain to the desired value
	
	ARGUMENTS:
	- tf: 				pointer to the transfer function
	- new_dcgain:	new desired steady-state gain
************************************************************************************************************************/
void set_dcgain(struct tf *tf, float new_dcgain);


/************************************************************************************************************************
	NAME: tf_reset_io
	
	DESCRIPTION: sets to 0 all the IO (inputs and outputs) of the given transfer function
								
	y[k] = y[k-1] = ... = y[k-n] = 0;
	u[k-n+m] = u[k-n+m-1] = ... = u[k-n] = 0;
************************************************************************************************************************/
void tf_reset_io(struct tf *tf);


/************************************************************************************************************************
	NAME: reset_integr_error_if_overshoot
	
	DESCRIPTION: if the reference have been overshot (and so the robot has to change its direction of movement) then reset the integral error area
	
	ARGUMENTS:
	- robot: 				pointer to the considered robot data structure
	- state:				index (e.g. 0, 1, 2 ...) of the robot's state of which we should reset the integral error (if it actually overshot the reference)
************************************************************************************************************************/
void reset_integr_error_if_overshoot(struct robot *robot, uint8_t state);
	

/************************************************************************************************************************
	NAME: save_prev_robot_state
	
	DESCRIPTION: save the robot state (e.g. x, u, ref, error) of previous iteration, in order to use it for the next iteration
	
	ARGUMENTS:
	- robot: 		pointer to the robot data structure of which you want to save the previous state
************************************************************************************************************************/
void save_prev_robot_state(struct robot *robot);


// TODO document this
void fir_init(struct fir *fir, uint32_t len, float *coeff, uint8_t is_feedback_from_filtered_data);

// TODO document this
float fir_filter(struct fir *fir, float new_raw);

// TODO document this
void fir_set(struct fir *fir, float new_raw);

// TODO document this
float fir_get(struct fir *fir);



/************************************************************************************************************************
	NAME: ref_yaw_chassis_dir
	
	DESCRIPTION: given a yaw reference for the chassis (w.r.t. the gimbal), returns +1 or -1 when, at the end of the maneuver, the chassis
								will be aligned towards that yaw reference with its forward of backward part (respectively), considering that it will take
								the shortest path to arrive to the yaw reference.
	
	ARGUMENTS:
	- ref_yaw:			reference yaw angle to be reached.			[rad]
	- start_yaw:		yaw angle from which the chassis starts to move to reach the yaw reference.		[rad]
************************************************************************************************************************/
int ref_yaw_chassis_dir(float ref_yaw, float start_yaw);


/************************************************************************************************************************
	NAME: ref_yaw_chassis_rot
	
	DESCRIPTION: given a yaw reference for the chassis (w.r.t. the gimbal), returns the yaw angle that the chassis has to reach in order to align
								to the given yaw reference in the fastest way, NO MATTER IF WITH FORWARD OR BACKWARD part of the chassis.
	
	ARGUMENTS:
	- ref_yaw:			reference yaw angle to be reached (by either the forward or backward part of the chassis).			[rad]
	- start_yaw:		yaw angle from which the chassis starts to move to reach the yaw reference.		[rad]
************************************************************************************************************************/
float ref_yaw_chassis_rot(float ref_yaw, float start_yaw);


// TODO document this
void cv_gimbal_auto_aim_init(struct cv_gimbal_auto_aim* cv, float v0, uint8_t yaw_filter_buf_size, uint8_t pitch_filter_buf_size, float* pitch_fit_coeff,
														 float min_shoot_distance, float max_shoot_distance, float max_shoot_freq, float max_enemy_lock_distance, float shoot_freq_enemy_warn);

// TODO document this
void cv_enemy_track(float* ref_yaw, float* ref_pitch, float* shoot_freq, struct cv_gimbal_auto_aim* cv, float curr_yaw, float curr_pitch);


// TODO document this
void sentry_state_transition(void);


#endif