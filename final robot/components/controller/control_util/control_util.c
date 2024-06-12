#include <math.h>
#include "control_util.h"
#include "CAN_receive.h"
#include "robot_config.h"
#include "remote_control.h"
#include "control_alg.h"
#include "bmi088driver.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "AI_receive.h"
#include "referee.h"


/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/* sentry's state machine */
uint8_t sentry_state = IDLE;						// current sentry's state (received from ROS), used to decide how the sentry should behave
uint8_t sentry_state_prev = IDLE;					// previous sentry's state
uint8_t is_sentry_spot_reached = TRUE;		// 1 if the sentry's chassis target on ground is reached (in MOVE_TO_SPOT state)
uint8_t is_cv_enemy_lost = TRUE;					// 1 if we have lost all enemies (i.e., the CV didn't detect any enemy) since a certain amount of time
uint8_t is_cv_enemy_stalled = FALSE;			// 1 if the detected enemy has been staying the middle of the short/long distance range for a certain amount of time
float time_ms_last_enemy_detected = 0;
float time_ms_enemy_began_to_stall = 0;
float time_enemy_lost = 1.5;							// time that should elapse to declare that we have lost all enemy robots
float time_enemy_stalled = 2.0;						// time that should elapse to declare that the enemy robot is stalling on the short/long distance range

/* long keys press weights */
float weight_fwd_key = 0.0;
float weight_left_key = 0.0;
float weight_bwd_key = 0.0;
float weight_right_key = 0.0;

/* mouse commands */
float mouse_speed_x;
float mouse_speed_y;
float mouse_speed_x_prev = 0.0;
float mouse_speed_y_prev = 0.0;

/*** gimbal mouse movement values ***/
uint32_t clock_ms_mouse_gimbal;
uint32_t clock_ms_mouse_gimbal_prev = 0;
float mouse_meters_yaw_rad_ratio = 0.016375;		// if mouse travels 0.01 meters, then yaw has to travel 35*pi/180 rad
float mouse_meters_pitch_rad_ratio = 0.01;			// if mouse travels 0.005 meters, then yaw has to travel 30*pi/180 rad

/* initial gain of yaw and pitch of gimbal */
float initial_gimbal_yaw_gain = 0.0;
float initial_gimbal_pitch_gain = 0.0;

/* brake BR movement */
int direction_BR_wheels_to_break = 0;

/* flags */
int shoot_motors_speed_increased = 0;
int mouse_sensibility_pitch_increased = 0;
int mouse_sensibility_yaw_increased = 0;
int theta_balance_value_increased = 0;
//int BR_movement_braked_successfully = 0;



/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void saturate(float *value, float bound) {
	
	bound = fabs(bound);		// consider the absolute value of the given bound (a negative bound wouldn't make sense)
	
	if (*value > bound)
		*value = bound;
	else if (*value < -bound)
		*value = -bound;
}


void saturate_in_range(float *value, float lb, float ub) {
	
	if (*value < lb)
		*value = lb;
	if (*value > ub)
		*value = ub;
}


/* generate a white noise signal with random gaussian distribution (mean=0, std_dev=1) */
void wn_gauss_gen(float *wn_gauss, unsigned int num_samples) {
	
	// NOTE: the memory for "wn_gauss" must be already allocated ("num_samples" entries of size "sizeof(float)")
	
	// randomly initialize the random seed (just to be sure)
	srand(HAL_GetTick());
	
	// TODO delete
//	unsigned int seed = 0;
//	srand(seed);
	
	float wn_mean = 0;			// white noise mean
	float wn_std_dev = 0;		// white noise standard deviation
	
	// generate the gaussian distributed numbers with the Box-Muller transform algorithm
	for (unsigned int i = 0; i < num_samples; i++) {
		
		// generate 2 indipendent uniformely distributed numbers
		float udistr_n1 = (float)rand() / (float)RAND_MAX;
		float udistr_n2 = (float)rand() / (float)RAND_MAX;
		
		// from the 2 uniformely distributed numbers, generate other 2 indipendent gaussian distributed numbers
		float gdistr_n1 = sqrt(-2*log(udistr_n1))*cos(2*pi*udistr_n2);
		float gdistr_n2 = sqrt(-2*log(udistr_n1))*sin(2*pi*udistr_n2);
		
		// store one of the 2 gaussian distributed numbers (discard the other one)
		wn_gauss[i] = gdistr_n1;
		
		// update the white noise mean
		wn_mean += gdistr_n1;
	}
	
	
	/* normalize the white noise to mean=0 and std_dev=1 */
	
	// 1: compute white noise mean
	wn_mean = wn_mean/num_samples;
	
	// 2: compute white noise standard deviation
	for (unsigned int i = 0; i < num_samples; i++) {
		
		wn_std_dev += pow((wn_gauss[i] - wn_mean), 2);
	}
	
	wn_std_dev = wn_std_dev/(num_samples-1);
	
	// 3: for each white noise sample, subtract to it the mean and divide it by the std_dev
	for (unsigned int i = 0; i < num_samples; i++) {
		
		wn_gauss[i] = (wn_gauss[i] - wn_mean) / wn_std_dev;
	}
}


void rand_bin_gen(float *v, unsigned int num_samples) {
	
	// randomly initialize the random seed (just to be sure)
	srand(HAL_GetTick());
	
	// TODO delete
//	unsigned int seed = 0;
//	srand(seed);
	
	// generate array of random values among the set {0,1}
	for (int i = 0; i < num_samples; i++) {
		v[i] = (float)((int)rand() % 2);
	}
}


void rand_unif_gen(float *v, unsigned int num_samples) {
	
	// randomly initialize the random seed (just to be sure)
	srand(HAL_GetTick());
	
	// TODO delete
//	unsigned int seed = 0;
//	srand(seed);
	
	// generate array of random values in range [0,1]
	for (int i = 0; i < num_samples; i++) {
		v[i] = (float)rand() / (float)RAND_MAX;
	}
}


void compute_weights_WASD_keys(float dt) {
	
	/* compute WASD keys weights */
#if IS_STD || IS_SENTRY
	
	weight_fwd_key 		+= (is_key_pressed(KEY_W)) ? dt : - dt;
	weight_left_key 	+= (is_key_pressed(KEY_A)) ? dt : - dt;
	weight_bwd_key 		+= (is_key_pressed(KEY_S)) ? dt : - dt;
	weight_right_key 	+= (is_key_pressed(KEY_D)) ? dt : - dt;
	
//	weight_fwd_key 		= (is_key_pressed(KEY_W)) ? weight_fwd_key 		+ dt 	: 0.0;
//	weight_left_key 	= (is_key_pressed(KEY_A)) ? weight_left_key 	+ dt 	: 0.0;
//	weight_bwd_key 		= (is_key_pressed(KEY_S)) ? weight_bwd_key 		+ dt 	: 0.0;
//	weight_right_key 	= (is_key_pressed(KEY_D)) ? weight_right_key 	+ dt 	: 0.0;
	
	
	
	
//	float add_fwd 		= (is_key_pressed(KEY_W)) ? dt/STD_TIME_RAISING_EDGE_WASD_KEYS : - dt/STD_TIME_FALLING_EDGE_WASD_KEYS;
//	float add_left 		= (is_key_pressed(KEY_A)) ? dt/STD_TIME_RAISING_EDGE_WASD_KEYS : - dt/STD_TIME_FALLING_EDGE_WASD_KEYS;
//	float add_bwd 		= (is_key_pressed(KEY_S)) ? dt/STD_TIME_RAISING_EDGE_WASD_KEYS : - dt/STD_TIME_FALLING_EDGE_WASD_KEYS;
//	float add_right 	= (is_key_pressed(KEY_D)) ? dt/STD_TIME_RAISING_EDGE_WASD_KEYS : - dt/STD_TIME_FALLING_EDGE_WASD_KEYS;

//	if (is_key_pressed(KEY_S) && weight_fwd_key > 0)
//		add_fwd *= (STD_TIME_FALLING_EDGE_WASD_KEYS/STD_TIME_FALLING_EDGE_BRAKE_WASD_KEYS);
//	if (is_key_pressed(KEY_D) && weight_left_key > 0)
//		add_left *= (STD_TIME_FALLING_EDGE_WASD_KEYS/STD_TIME_FALLING_EDGE_BRAKE_WASD_KEYS);
//	if (is_key_pressed(KEY_W) && weight_bwd_key > 0)
//		add_bwd *= (STD_TIME_FALLING_EDGE_WASD_KEYS/STD_TIME_FALLING_EDGE_BRAKE_WASD_KEYS);
//	if (is_key_pressed(KEY_A) && weight_right_key > 0)
//		add_right *= (STD_TIME_FALLING_EDGE_WASD_KEYS/STD_TIME_FALLING_EDGE_BRAKE_WASD_KEYS);
//	
//	weight_fwd_key		+= add_fwd;
//	weight_left_key		+= add_left;
//	weight_bwd_key		+= add_bwd;
//	weight_right_key	+= add_right;
#elif IS_BR2
//	weight_fwd_key 		= (is_key_pressed(KEY_W)) ? weight_fwd_key + dt 		: 0.0;
//	weight_left_key 	= (is_key_pressed(KEY_A)) ? weight_left_key + dt 		: 0.0;
//	weight_bwd_key 		= (is_key_pressed(KEY_S)) ? weight_bwd_key + dt 		: 0.0;
//	weight_right_key 	= (is_key_pressed(KEY_D)) ? weight_right_key + dt 	: 0.0;
	
	
	float add_fwd 		= (is_key_pressed(KEY_W)) ? dt/BR_TIME_RAISING_EDGE_WASD_KEYS : - dt/BR_TIME_FALLING_EDGE_WASD_KEYS;
	float add_left 		= (is_key_pressed(KEY_A)) ? dt/BR_TIME_RAISING_EDGE_WASD_KEYS : - dt/BR_TIME_FALLING_EDGE_WASD_KEYS;
	float add_bwd 		= (is_key_pressed(KEY_S)) ? dt/BR_TIME_RAISING_EDGE_WASD_KEYS : - dt/BR_TIME_FALLING_EDGE_WASD_KEYS;
	float add_right 	= (is_key_pressed(KEY_D)) ? dt/BR_TIME_RAISING_EDGE_WASD_KEYS : - dt/BR_TIME_FALLING_EDGE_WASD_KEYS;
	
//	if (is_key_pressed(KEY_W) && weight_bwd_key > 0)
//		add_fwd *= (BR_TIME_RAISING_EDGE_WASD_KEYS/BR_TIME_RAISING_EDGE_BRAKE_WASD_KEYS);
//	if (is_key_pressed(KEY_A) && weight_right_key > 0)
//		add_left *= (BR_TIME_RAISING_EDGE_WASD_KEYS/BR_TIME_RAISING_EDGE_BRAKE_WASD_KEYS);
//	if (is_key_pressed(KEY_S) && weight_fwd_key > 0)
//		add_bwd *= (BR_TIME_RAISING_EDGE_WASD_KEYS/BR_TIME_RAISING_EDGE_BRAKE_WASD_KEYS);
//	if (is_key_pressed(KEY_D) && weight_left_key > 0)
//		add_right *= (BR_TIME_RAISING_EDGE_WASD_KEYS/BR_TIME_RAISING_EDGE_BRAKE_WASD_KEYS);

	if (is_key_pressed(KEY_S) && weight_fwd_key > 0)
		add_fwd *= (BR_TIME_FALLING_EDGE_WASD_KEYS/BR_TIME_FALLING_EDGE_BRAKE_WASD_KEYS);
	if (is_key_pressed(KEY_D) && weight_left_key > 0)
		add_left *= (BR_TIME_FALLING_EDGE_WASD_KEYS/BR_TIME_FALLING_EDGE_BRAKE_WASD_KEYS);
	if (is_key_pressed(KEY_W) && weight_bwd_key > 0)
		add_bwd *= (BR_TIME_FALLING_EDGE_WASD_KEYS/BR_TIME_FALLING_EDGE_BRAKE_WASD_KEYS);
	if (is_key_pressed(KEY_A) && weight_right_key > 0)
		add_right *= (BR_TIME_FALLING_EDGE_WASD_KEYS/BR_TIME_FALLING_EDGE_BRAKE_WASD_KEYS);
	
	weight_fwd_key		+= add_fwd;
	weight_left_key		+= add_left;
	weight_bwd_key		+= add_bwd;
	weight_right_key	+= add_right;
	
	
	
//	if (!is_key_pressed(KEY_W) && weight_fwd_key > 0.5)
//		weight_fwd_key = 0.0;
//	if (!is_key_pressed(KEY_A) && weight_left_key > 0.5)
//		weight_left_key = 0.0;
//	if (!is_key_pressed(KEY_S) && weight_bwd_key > 0.5)
//		weight_bwd_key = 0.0;
//	if (!is_key_pressed(KEY_D) && weight_right_key > 0.5)
//		weight_right_key = 0.0;
		
#endif
	
	/* saturate WASD keys weights between 0 and 1 */
	weight_fwd_key 		= max(min(weight_fwd_key, 	1.0), 0.0);
	weight_left_key 	= max(min(weight_left_key, 	1.0), 0.0);
	weight_bwd_key 		= max(min(weight_bwd_key, 	1.0), 0.0);
	weight_right_key 	= max(min(weight_right_key, 1.0), 0.0);
}


void update_mouse_position_gimbal(float sample_period, float *cmd_yaw, float *cmd_pitch) {
	
	clock_ms_mouse_gimbal = clock_ms;
	float dt = clock_ms_mouse_gimbal - clock_ms_mouse_gimbal_prev;
	
	mouse_speed_x = ((float) get_remote_control_point()->mouse.x) * 0.001;	// 0.001 = millimeter/meter
	mouse_speed_y = ((float) get_remote_control_point()->mouse.y) * 0.001;
	
	saturate(&mouse_speed_x, MAX_MOUSE_SPEED_X);
	saturate(&mouse_speed_y, MAX_MOUSE_SPEED_Y);
	
	*cmd_yaw -= ((mouse_speed_x_prev + (mouse_speed_x-mouse_speed_x_prev)/2) * sample_period)/mouse_meters_yaw_rad_ratio;
	*cmd_pitch -= ((mouse_speed_y_prev + (mouse_speed_y-mouse_speed_y_prev)/2) * sample_period)/mouse_meters_pitch_rad_ratio;
	
	mouse_speed_x_prev = mouse_speed_x;
	mouse_speed_y_prev = mouse_speed_y;
	
	clock_ms_mouse_gimbal_prev = clock_ms_mouse_gimbal;
}


void check_keyboard_commands_shoot_motors(uint8_t shooting_control_mode) {
	
	if (shooting_control_mode == NO_SHOOT_MODE)		// check, just to be sure
		return;
	
//	// set reference speed for shoot motors
//	if (is_key_raising_edge(KEY_Q) && !shoot_motors_speed_increased) {
//		ref_shoot_motors_speed_radsec += (is_key_pressed(KEY_CTRL)) ? -50 : 50;
//		ref_shoot_motors_speed_radsec = min(max(ref_shoot_motors_speed_radsec, 0), 800);	// mechanical speed limit of shoot motors is around 920 rad/s
//		shoot_motors_speed_increased = 1;
//	}
//	else if (is_key_falling_edge(KEY_Q)) {
//		shoot_motors_speed_increased = 0;
//	}
}


void shoot_model_init(shoot_motors_model_t *model) {
	
	for (int i = 0; i < 4; i++) {
		model->ref[i] = 0;
		model->ref_prev[i] = 0;
		model->error[i] = 0;
		model->error_prev[i] = 0;
		model->integral_error[i] = 0;
	}
}

void rev_model_init(rev_motor_model_t *model) {
	
	for (int i = 0; i < 2; i++) {
		model->ref[i] = 0;
		model->ref_prev[i] = 0;
		model->error[i] = 0;
		model->error_prev[i] = 0;
		model->integral_error[i] = 0;
	}
}


void robot_init(struct robot *robot, uint8_t n, uint8_t m) {

	/* set number of robot states and inputs */
	
	robot->n = n;			// states
	robot->m = m;			// inputs
	
	/* reset the given "robot" data structure */
	
	for (int i = 0; i < n; i++) {
		
		robot->x[i] 							= 0;
		robot->x_prev[i] 					= 0;
		robot->ref[i] 						= 0;
		robot->ref_prev[i] 				= 0;
		robot->error[i] 					= 0;
		robot->error_prev[i] 			= 0;
		robot->integr_error[i] 		= 0;
		robot->deriv_error[i]			= 0;
	}
	
	for (int i = 0; i < m; i++) {
		
		robot->u[i] 			= 0;
		robot->u_prev[i] 	= 0;
	}
}




void tf_init(struct tf *tf, float dt, uint8_t m, uint8_t n, float *num, float *den) {	
	
	tf->dt = dt;	// set tf sampling time
	tf->m = m;		// set tf numerator order
	tf->n = n;		// set tf denominator order
	
	/* set tf numerator/denominator and reset all tf inputs/outputs */
	for (int i = 0; i <= n; i++) {
		tf->num[i] = (i < n-m) ? 0 : num[i-n+m];
		tf->den[i] = den[i];
		tf->u[i] = 0;
    tf->y[i] = 0;
  }
}

/*
void zpk2tf(uint8_t m, uint8_t n, float complex *z, float complex *p, float k, float *num, float *den) {

    // compute the tf numerator
    roots2coeff(m, z, num);

    // compute the tf denominator
    roots2coeff(n, p, den);

    // multiply the tf numerator by the tf gain
    for (int i = 0; i <= m; i++)
        num[i] *= k;
}
*/

void data_sampling_init(struct data_sample *ds, uint32_t num_samples, float dt) {
	
	ds->num_samples = num_samples;		// desired number of data to be sampled
	ds->dt = dt;											// desired sample period (in seconds)
	ds->k = 0;												// reset samples counter
	ds->clock_tick_ms_last_iter = 0;	// reset clock time of last iteration
}


void acquire_data_sample(struct data_sample *ds, float sample) {
	
	// get current clock time (in milliseconds)
	unsigned long clock_tick_ms_curr_iter = HAL_GetTick();
	
	/*
	*		if...
	*			- we didn't reach the desired number of sampled data yet
	*			- at least "dt" seconds passed since the last data sample
	*		then acquire the new sample
	*/
	int should_acquire_sample = (ds->k < ds->num_samples) && (clock_tick_ms_curr_iter - ds->clock_tick_ms_last_iter >= (ds->dt)*1e3);
	
	if (should_acquire_sample) {
		
		ds->samples[ds->k] = sample;														// acquire the sample
		ds->clock_tick_ms_last_iter = clock_tick_ms_curr_iter;	// save current clock tick (for next iteration)
		(ds->k)++;																							// increase samples counter
	}
}


float send_data_sample(struct data_sample *ds) {
	
	float value_to_send;
	
	// get current clock time (in milliseconds)
	unsigned long clock_tick_ms_curr_iter = HAL_GetTick();
	
	/*
	*		if...
	*			- first sample of the sequence still has to be sent
	*		or...
	*			- we didn't reach the desired number of sent data yet
	*			- at least "dt" seconds passed since the last sent data
	*		then send a new sample, otherwise send the previous one
	*/
	int should_return_next_sample = (ds->k == 0) || ((ds->k < ds->num_samples) && (clock_tick_ms_curr_iter - ds->clock_tick_ms_last_iter >= (ds->dt)*1e3));
	
	if (should_return_next_sample) {
		
		ds->clock_tick_ms_last_iter = clock_tick_ms_curr_iter;	// save current clock tick (for next iteration)
		value_to_send = ds->samples[ds->k];											// send new sample
		(ds->k)++;																							// increase samples counter
	}
	else if (ds->k == ds->num_samples) {
		
		value_to_send = 0;																			// send a null control signal (the samples sequence is finished)
	}
	else {
		
		value_to_send = ds->samples[ds->k - 1];									// send previous sample
	}
	
	return value_to_send;
}


float dcgain(struct tf *tf) {

	float sum_num = 0;
	float sum_den = 0;
	uint8_t m = tf->m;
	uint8_t n = tf->n;

	for (int i = n-m; i <= n; i++)
			sum_num += tf->num[i];			// compute the sum of all tf numerator coefficients
	for (int i = 0; i <= n; i++)
			sum_den += tf->den[i];			// compute the sum of all tf denominator coefficients

	return sum_num/sum_den;					// return the tf steady-state gain
}


void set_dcgain(struct tf *tf, float new_dcgain) {

    float old_dcgain = dcgain(tf);		// compute the old tf steady-state gain
    uint8_t m = tf->m;
    uint8_t n = tf->n;

    for (int i = n-m; i <= n; i++)
        tf->num[i] *= (new_dcgain / old_dcgain);		// update the tf steady-state gain
}


void tf_reset_io(struct tf *tf) {

    for (int i = 0; i <= tf->m; i++) {
        tf->u[i] = 0;		// reset all tf inputs
        tf->y[i] = 0;		// reset all tf outputs
    }
}


void reset_integr_error_if_overshoot(struct robot *robot, uint8_t state) {
	
	if (robot->error[state] * robot->error_prev[state] <= 0)
				robot->integr_error[state] = 0;
}


void save_prev_robot_state(struct robot *robot) {
	
	for (int i = 0; i < robot->n; i++) {
		
		robot->x_prev[i] = robot->x[i];
		robot->ref_prev[i] = robot->ref[i];
		robot->error_prev[i] = robot->error[i];
	}
	
	for (int i = 0; i < robot->m; i++) {
		
		robot->u_prev[i] = robot->u[i];
	}
}


void fir_init(struct fir *fir, uint32_t len, float *coeff, uint8_t is_feedback_from_filtered_data) {
	
	fir->len = len;																														// set filter's length
	fir->is_feedback_from_filtered_data = is_feedback_from_filtered_data;			// set filter working mode
	
	for (int i = 0; i < len; i++) {
		fir->coeff[i] = coeff[i];			// set filter's coefficients
		fir->buf[i] = 0;							// reset filter's buffer
	}
	
	fir->buf_offset = 0;						// reset filter's pointer to the newest elements
}


float fir_filter(struct fir *fir, float new_raw) {
	
	float out;								// filtered output
	
	fir_set(fir, new_raw);		// update the filter with the newest raw value
	out = fir_get(fir);				// apply the filter to the values inside the filter's buffer
	
	if (fir->is_feedback_from_filtered_data) {
		
		fir->buf[fir->buf_offset] = out;			// insert inside the FIR buffer the filtered value, so that it will be used as one of the "previous" values in the next iterations
	}

	return out;
}
	

void fir_set(struct fir *fir, float new_raw) {
	
	// check that a FIR filter was actually provided as argument
	if (fir == NULL)
		return;
	
	uint32_t len = fir->len;			// get filter's length
	
	/* update the buffer's offset to the newest entry */
	fir->buf_offset--;						// point to next entry (going from the end to the beginning of the buffer)
	if (fir->buf_offset < 0)			// if the pointer exceeded the circular buffer...
		fir->buf_offset = len-1;		// ... then start again from the buffer's end
	
	fir->buf[fir->buf_offset] = new_raw;		// insert the newest filter input in the buffer
	
	
//	for (int i = lpf->n - 1; i > 0; i--) {
//		lpf->buffer[i] = lpf->buffer[i-1];
//	}
//	
//	lpf->buffer[0] = new_raw;
}


float fir_get(struct fir *fir) {
	
	// check that a FIR filter was actually provided as argument
	if (fir == NULL)
		return 0;
	
	uint32_t 	len	= fir->len;						// get filter's length
	int 			off	= fir->buf_offset;		// get offset of filter's buffer
	
	/* compute the filtered output (as the weighted average of buffer entries, which weights are the filter's coefficients) */
	float out = 0;
	for (int i = 0; i < len; i++) {
		out += fir->coeff[i] * fir->buf[(i+off) % len];
	}
	
	return out;
}


int ref_yaw_chassis_dir(float ref_yaw, float start_yaw) {
	
	// NOTE: start_yaw represents the angle where the chassis' forward part is currently aligned to.
	
	// cast the yaw reference as an (unbounded) contiguous angle
	ref_yaw = nearest_contig_ref_angle(ref_yaw, start_yaw);
	
	if (fabs(ref_yaw - start_yaw) <= pi/2)
		return +1;
	
	return -1;
}


float ref_yaw_chassis_rot(float ref_yaw, float start_yaw) {
	
	// NOTE: start_yaw represents the angle where the chassis' forward part is currently aligned to.
	
	// cast the yaw reference as an (unbounded) contiguous angle
	ref_yaw = nearest_contig_ref_angle(ref_yaw, start_yaw);
	
	if (fabs(ref_yaw - start_yaw) <= pi/2)
		return ref_yaw;			// reach with chassis' forward part
	
	// reach with chassis' backward part
	ref_yaw += pi;
	ref_yaw = nearest_contig_ref_angle(ref_yaw, start_yaw);
	
	return ref_yaw;
}


void cv_gimbal_auto_aim_init(struct cv_gimbal_auto_aim* cv, float v0, uint8_t yaw_filter_buf_size, uint8_t pitch_filter_buf_size, float* pitch_fit_coeff,
														 float min_shoot_distance, float max_shoot_distance, float max_shoot_freq, float max_enemy_lock_distance, float shoot_freq_enemy_warn) {
	
	// check correctness of input arguments
	if (cv == NULL || yaw_filter_buf_size <= 0 || pitch_filter_buf_size <= 0 || pitch_fit_coeff == NULL)
		return;
	
	/* variables assignments */
	cv->dt = 0.0;
	cv->v0 = v0;
	cv->clock_ms_prev_iter = 0;
	cv->h_shooter = 0.35;					// shooter height from ground [m]
	cv->distance_left_cam_to_center_zed2 = 0.06;								// distance (in absolute value) between camera sx and center of ZED2 stereocamera
	cv->distance_center_zed2_to_center_shooting = 0.045;				// vertical distance (in absolute value) between the ZED2's center and the shooter
	cv->distance_left_cam_to_center_zed_mini = 0.0325;					// distance (in absolute value) between camera sx and center of ZED Mini stereocamera
	cv->distance_center_zed_mini_to_center_shooting = 0.04;			// vertical distance (in absolute value) between the ZED Mini's center and the shooter
	cv->yaw_filter_buf_size = yaw_filter_buf_size;
	cv->pitch_filter_buf_size = pitch_filter_buf_size;
	cv->min_shoot_distance = min_shoot_distance;
	cv->max_shoot_distance = max_shoot_distance;
	cv->max_enemy_lock_distance = max_enemy_lock_distance;
	cv->max_shoot_freq = max_shoot_freq;
	cv->shoot_freq_enemy_warn = shoot_freq_enemy_warn;
	cv->d_armor = 0.0;      // distance from enemy's armor (on ground, i.e., along Y axis)		[m]
	cv->h_armor = 0.0;      // altitude of enemy's armor (from ground)		[m]
	cv->d_xyz = 0.0;				// Euclidean distance to the enemy's armor
	cv->first_iter_done = FALSE;
	
	/* reset stereo camera values */
	cv->x_cam_left = 0;
	cv->y_cam_left = 0;
	cv->z_cam_left = 0;
	cv->x_cam_left_prev = 0;
	cv->y_cam_left_prev = 0;
	cv->z_cam_left_prev = 0;
	
	/* initialize coefficients of yaw/pitch filters */
	float yaw_filter_coeff[yaw_filter_buf_size];
	float pitch_filter_coeff[pitch_filter_buf_size];
	
	for (int i = 0; i < yaw_filter_buf_size; i++)
		yaw_filter_coeff[i] = 1/((float)yaw_filter_buf_size);
	
	for (int i = 0; i < pitch_filter_buf_size; i++)
		pitch_filter_coeff[i] = 1/((float)pitch_filter_buf_size);
	
	/* initialize yaw/pitch filters */
	fir_init(&(cv->fir_yaw_cv), 	yaw_filter_buf_size, 	 yaw_filter_coeff, 	 FALSE);
	fir_init(&(cv->fir_pitch_cv), pitch_filter_buf_size, pitch_filter_coeff, FALSE);
	
	/* initialize data fitting coefficients of CV-driven pitch auto-aim */
	for (int i = 0; i < CV_PITCH_FIT_COEFFS; i++)
		cv->pitch_fit_coeff[i] = pitch_fit_coeff[i];
}
														 

void cv_enemy_track(float* ref_yaw, float* ref_pitch, float* shoot_freq, struct cv_gimbal_auto_aim* cv, float curr_yaw, float curr_pitch) {
	
	/* outputs to be computed:
	*  ref_yaw 		==> gimbal's yaw reference angle with respect to the current yaw position
	*  ref_pitch 	==> gimbal's pitch reference angle with respect to the horizon
	*	 shoot_freq	==> shooting frequency (depends on how far is the detected enemy)
	*/
	
	/*		  Frame reference intrinsic of ZED Mini and ZED2:
	*
	*		    								^ Z
	*		    								|
	*		    								|
	*		    	+------------------------------+
	*		    	|		  __					 __					 |		 Y
	*		    	|    |__|					|__|    ZED	 |---->
	*		    	|		 													 |
	*		    	+------------------------------+
	*		  	  						 (o)
	*		    							X
	*
	*/
	
	/*		    Frame reference that we use for CV algorithms:
	*
	*		    								^ Z
	*		    								|
	*	    									|
	*		    	+------------------------------+
	*		    	|		  __					 __					 |
	*	   <----|    |__|					|__|    ZED	 |
	*		X    	|		 													 |
	*		  	  +------------------------------+
	*		  	  						 (o)
	*			    						Y
	*
	*/
	
	/* get clock time info */
	cv->clock_ms_curr_iter = HAL_GetTick();
	if (!cv->first_iter_done)
		cv->clock_ms_prev_iter = cv->clock_ms_curr_iter;
	
	// time period elapsed between the current CV detection and the previous one	[s]
	cv->dt = ((float)(cv->clock_ms_curr_iter - cv->clock_ms_prev_iter)) * 1e-3;
	
	// get enemy's x,y,z coordinates from stereo camera
	cv->x_cam_left = - get_AI_ref1();
	cv->y_cam_left = + get_AI_ref0();
	cv->z_cam_left = + get_AI_ref2();

	// add the offsets due to the ZED's physical structure
	cv->x_cam = cv->x_cam_left - cv->distance_left_cam_to_center_zed2;
	cv->y_cam = cv->y_cam_left;
	cv->z_cam = cv->z_cam_left + cv->distance_center_zed2_to_center_shooting;
	
	// yaw/pitch references obtained by the CV
	float ref_yaw_cv;
	float ref_pitch_cv;

	/* compute yaw/pitch references with respect to the current yaw/pitch angle */
	if (cv->x_cam_left == 0 && cv->y_cam_left == 0 && cv->z_cam_left == 0) {
		
		/* ZED detection never found any enemy yet */
		ref_yaw_cv 	 = 0;
		ref_pitch_cv = 0;
	}
	else {
		
		/* ZED detected an enemy, so we have to aim to it */
		
		// point yaw towards the enemy
		ref_yaw_cv = atan2(- cv->x_cam / sqrt(cv->x_cam*cv->x_cam + cv->y_cam*cv->y_cam), + cv->y_cam / sqrt(cv->x_cam*cv->x_cam + cv->y_cam*cv->y_cam));
		
		// estimate the horizontal/vertical distance between sentry/enemy's armor and enemy's armor/ground
		cv->d_armor = cv->y_cam*cos(curr_pitch) - cv->z_cam*sin(curr_pitch);                  		// distance from enemy's armor (on ground, i.e., along Y axis)
		cv->h_armor = cv->h_shooter + cv->y_cam*sin(curr_pitch) + cv->z_cam*cos(curr_pitch);      // altitude of enemy's armor (from ground)

#if ENABLE_CV_AUTO_AIM_PITCH_PRED
		// set pitch reference such that it accounts for gravity on bullets
		ref_pitch_cv 	= cv->pitch_fit_coeff[0] +
										cv->d_armor * cv->pitch_fit_coeff[1] +
										cv->h_armor * cv->pitch_fit_coeff[2] +
										cv->d_armor * cv->d_armor * cv->pitch_fit_coeff[3] +
										cv->d_armor * cv->h_armor * cv->pitch_fit_coeff[4] +
										cv->h_armor * cv->h_armor * cv->pitch_fit_coeff[5];
#else
		// just track the enemy, WITHOUT bullets' gravity pitch compensation
		ref_pitch_cv = atan2(+ cv->z_cam / sqrt(cv->z_cam*cv->z_cam + cv->y_cam*cv->y_cam), + cv->y_cam / sqrt(cv->z_cam*cv->z_cam + cv->y_cam*cv->y_cam));
#endif
	}
	
	
	/* set the reference signals */
	if (cv->x_cam_left != cv->x_cam_left_prev || cv->y_cam_left != cv->y_cam_left_prev || cv->z_cam_left != cv->z_cam_left_prev) {
		
		// a new info just came from the stereocamera, so update the reference signal
		*ref_yaw 	 = curr_yaw + ref_yaw_cv;
		*ref_pitch = ref_pitch_cv;
		
		// update the previous x,y,z values obtained from the stereocamera
		cv->x_cam_left_prev = cv->x_cam_left;
		cv->y_cam_left_prev = cv->y_cam_left;
		cv->z_cam_left_prev = cv->z_cam_left;
	}
	
	/* for both yaw and pitch, check if the value received from ZED detection is NaN; if yes, then substitute it with the last not-NaN receive value */
	if (isnan(*ref_yaw) || isinf(*ref_yaw))
		*ref_yaw = cv->fir_yaw_cv.buf[cv->fir_yaw_cv.buf_offset];
	if (isnan(*ref_pitch) || isinf(*ref_pitch))
		*ref_pitch = cv->fir_pitch_cv.buf[cv->fir_pitch_cv.buf_offset];
	
	/* filter the yaw/pitch references (obtained by the ZED with a low-pass filter */
	*ref_yaw 	 = fir_filter(&(cv->fir_yaw_cv), 	 *ref_yaw);
	*ref_pitch = fir_filter(&(cv->fir_pitch_cv), *ref_pitch);
	
	
	// compute the shooting frequency (based on the enemy distance from our robot: the more the enemy is far away, the lower will be the shooting frequency)
	cv->d_xyz = sqrt(cv->x_cam*cv->x_cam + cv->y_cam*cv->y_cam + cv->z_cam*cv->z_cam);   // distance from enemy's armor (in the 3D space)

	if ( cv->d_xyz >= cv->max_shoot_distance || sentry_state == ENEMY_LOCK || isnan(cv->d_xyz) || isinf(cv->d_xyz) ||
		  (cv->x_cam_left == cv->x_cam_left_prev && cv->y_cam_left == cv->y_cam_left_prev && cv->z_cam_left == cv->z_cam_left_prev)) {

		*shoot_freq = 0;
	}
	else if (sentry_state == ENEMY_WARN) {
		
		*shoot_freq = cv->shoot_freq_enemy_warn;
	}
	else if (cv->d_xyz <= cv->min_shoot_distance) {

		*shoot_freq = cv->max_shoot_freq;
	}
	else {

		*shoot_freq = (1 - (cv->d_xyz - cv->min_shoot_distance)/(cv->max_shoot_distance - cv->min_shoot_distance)) * cv->max_shoot_freq;
	}
	
	// save current clock time tick (for next iteration)
	cv->clock_ms_prev_iter = cv->clock_ms_curr_iter;
	
	// the first iteration for sure is completed
	cv->first_iter_done = TRUE;
}


void sentry_state_transition() {
	
	if (switch_is_down(rc_right_switch)) {
		
		// save previous sentry's state
		sentry_state_prev = sentry_state;
		
		return;
	}
	else if (switch_is_up(rc_right_switch)) {
		
		// if we command the sentry with the RC, we don't care about the sentry state, so we put it to ENEMY_ENGAGE (so that it can shoot)
		sentry_state = ENEMY_ENGAGE;
	}
	else if (get_game_progress() != IN_COMBAT) {			// NOTE: we enter here only if the remote controller's right switch is "middle"
		
		// starting from whatever state, the sentry should return to IDLE state if the match is not ON anymore
		sentry_state = IDLE;
	}
	else {		// NOTE: we enter here only if the remote controller's right switch is "middle" and the game progress is "IN_COMBAT"
		
		
		
		// TODO implement counters for is_sentry_spot_reached
		
		
		/* check if we lost the enemy that we were aiming at (or if we didn't find an enemy at all) */
		
		if (is_cv_enemy_lost) {
			
			if (cv_gimbal.x_cam_left != cv_gimbal.x_cam_left_prev || cv_gimbal.y_cam_left != cv_gimbal.y_cam_left_prev || cv_gimbal.z_cam_left != cv_gimbal.z_cam_left_prev) {
				
				// the CV detected an enemy
				is_cv_enemy_lost = FALSE;
				time_ms_last_enemy_detected = HAL_GetTick();
			}
		}
		else {
			
			if (cv_gimbal.x_cam_left == cv_gimbal.x_cam_left_prev && cv_gimbal.y_cam_left == cv_gimbal.y_cam_left_prev && cv_gimbal.z_cam_left == cv_gimbal.z_cam_left_prev) {
				
				if (HAL_GetTick() - time_ms_last_enemy_detected >= time_enemy_lost*1e3) {
					
					// we lost the enemy we were pointing at (or we didn't find any enemy yet)
					is_cv_enemy_lost = TRUE;
				}
			}
			else {
				
				// we are already aiming at an enemy
				time_ms_last_enemy_detected = HAL_GetTick();
			}
		}
		
		
		/* check if the enemy robot is stalled */
		
		if (is_cv_enemy_stalled) {
			
			if (!is_cv_enemy_lost && (cv_gimbal.d_xyz < cv_gimbal.max_shoot_distance || cv_gimbal.d_xyz >= cv_gimbal.max_enemy_lock_distance)) {
				
				// the CV detected an enemy
				is_cv_enemy_stalled = FALSE;
				time_ms_enemy_began_to_stall = HAL_GetTick();
			}
		}
		else {
			
			if (!is_cv_enemy_lost && cv_gimbal.d_xyz >= cv_gimbal.max_shoot_distance && cv_gimbal.d_xyz < cv_gimbal.max_enemy_lock_distance) {
				
				if (HAL_GetTick() - time_ms_enemy_began_to_stall >= time_enemy_stalled*1e3) {
					
					// we lost the enemy we were pointing at (or we didn't find any enemy yet)
					is_cv_enemy_stalled = TRUE;
				}
			}
			else {
				
				// we are already aiming at an enemy
				time_ms_enemy_began_to_stall = HAL_GetTick();
			}
		}
		
		
		/* state transitions */
		
		switch (sentry_state) {
			
			case IDLE: //
				
				if (!is_sentry_spot_reached)
					sentry_state = MOVE_TO_SPOT;
				
				else if (is_sentry_spot_reached)
					sentry_state = PATROL;
				
				break;
				
			case MOVE_TO_SPOT: //
				
				if (is_sentry_spot_reached)
					sentry_state = PATROL;
				
				break;
				
			case PATROL: //
				
				if (!is_cv_enemy_lost && cv_gimbal.d_xyz >= cv_gimbal.max_shoot_distance && cv_gimbal.d_xyz < cv_gimbal.max_enemy_lock_distance)
					sentry_state = ENEMY_LOCK;
				
				else if (!is_cv_enemy_lost && cv_gimbal.d_xyz < cv_gimbal.max_shoot_distance)
					sentry_state = ENEMY_ENGAGE;
				
				break;
			
			case ENEMY_LOCK: //
				
				if (!is_cv_enemy_lost && cv_gimbal.d_xyz < cv_gimbal.max_shoot_distance)
						sentry_state = ENEMY_ENGAGE;
				
				else if (!is_cv_enemy_lost && cv_gimbal.d_xyz >= cv_gimbal.max_shoot_distance && cv_gimbal.d_xyz < cv_gimbal.max_enemy_lock_distance && is_cv_enemy_stalled)
						sentry_state = ENEMY_WARN;
				
				else if (is_cv_enemy_lost || cv_gimbal.d_xyz >= cv_gimbal.max_enemy_lock_distance)
						sentry_state = PATROL;
				
				break;
			
			case ENEMY_WARN: //
				
				if (!is_cv_enemy_lost && cv_gimbal.d_xyz < cv_gimbal.max_shoot_distance)
						sentry_state = ENEMY_ENGAGE;
				
				else if (is_cv_enemy_lost || cv_gimbal.d_xyz >= cv_gimbal.max_enemy_lock_distance)
						sentry_state = PATROL;
					
				break;
			
			case ENEMY_ENGAGE: //
				
				if (!is_cv_enemy_lost && cv_gimbal.d_xyz >= cv_gimbal.max_shoot_distance && cv_gimbal.d_xyz < cv_gimbal.max_enemy_lock_distance) {
						sentry_state = ENEMY_LOCK;
					
				} else if (is_cv_enemy_lost || cv_gimbal.d_xyz >= cv_gimbal.max_enemy_lock_distance) {
						sentry_state = PATROL;
				}
				break;
			
		}
	}
	
	// save previous sentry's state
	sentry_state_prev = sentry_state;
}




