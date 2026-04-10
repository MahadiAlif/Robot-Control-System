#include <math.h>
#include "control_util.h"
#include "CAN_receive.h"
#include "robot_config.h"
#include "remote_control.h"
#include "control_alg.h"
#include "bmi088driver.h"
#include "cmsis_os.h"
#include "INS_task.h"


/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/* long keys press weights */
float weight_fwd_key = 0.0;
float weight_left_key = 0.0;
float weight_bwd_key = 0.0;
float weight_right_key = 0.0;

/* mouse's speed at current and previous instant of time (used for integration) */
float mouse_speed_x;
float mouse_speed_y;
float mouse_speed_x_prev = 0.0;
float mouse_speed_y_prev = 0.0;

/* initial gain of yaw and pitch of gimbal */
float initial_gimbal_yaw_gain = 0.0;
float initial_gimbal_pitch_gain = 0.0;

/* brake BR movement */
int direction_BR_wheels_to_break = 0;

/* flags */
int shoot_motors_speed_increased = 0;
int gain_LQR_pitch_increased = 0;
int gain_LQR_yaw_increased = 0;
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


void decide_if_mantain_gimbal_position(void) {
	
	// Yaw
	if (!need_to_mantain_gimbal_yaw_position &&
			(fabs(standard_gimbal.ref[0] - standard_gimbal.state_estim[0]) > fabs(standard_gimbal.ref[0] - standard_gimbal.state_estim_prev[0])) &&
			 fabs(standard_gimbal.ref[0] - standard_gimbal.state_estim[0]) > range_where_trigger_yaw_integral)
	{
		need_to_mantain_gimbal_yaw_position = 1;
		yaw_angle_where_mantain_position_starts = standard_gimbal.state_estim_prev[0];
		saturation_control_signal_yaw_gimbal = MAX_GIMBAL_YAW_MANTAIN_POSITION_CONTROL_SIGNAL_AMPLITUDE;
	}
	else if (fabs(standard_gimbal.ref[0] - standard_gimbal.state_estim[0]) <= fabs(standard_gimbal.ref[0] - standard_gimbal.state_estim_prev[0]))
	{
		need_to_mantain_gimbal_yaw_position = 0;
		saturation_control_signal_yaw_gimbal = MAX_GIMBAL_YAW_CONTROL_SIGNAL_AMPLITUDE;
	}
	
	// Pitch
	if (!need_to_mantain_gimbal_pitch_position &&
			(fabs(standard_gimbal.ref[1] - standard_gimbal.state_estim[1]) > fabs(standard_gimbal.ref[1] - standard_gimbal.state_estim_prev[1])) &&
			 fabs(standard_gimbal.ref[1] - standard_gimbal.state_estim[1]) > range_where_trigger_pitch_integral)
	{
		need_to_mantain_gimbal_pitch_position = 1;
		pitch_angle_where_mantain_position_starts = standard_gimbal.state_estim_prev[1];
		saturation_control_signal_pitch_gimbal = MAX_GIMBAL_PITCH_MANTAIN_POSITION_CONTROL_SIGNAL_AMPLITUDE;
	}
	else if (fabs(standard_gimbal.ref[1] - standard_gimbal.state_estim[1]) <= fabs(standard_gimbal.ref[1] - standard_gimbal.state_estim_prev[1]))
	{
		need_to_mantain_gimbal_pitch_position = 0;
		saturation_control_signal_pitch_gimbal = MAX_GIMBAL_PITCH_CONTROL_SIGNAL_AMPLITUDE;
	}
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
#if IS_STD
	weight_fwd_key 		+= (is_key_pressed(KEY_W)) ? dt : - dt;
	weight_left_key 	+= (is_key_pressed(KEY_A)) ? dt : - dt;
	weight_bwd_key 		+= (is_key_pressed(KEY_S)) ? dt : - dt;
	weight_right_key 	+= (is_key_pressed(KEY_D)) ? dt : - dt;
#elif IS_BR1 || IS_BR2
	weight_fwd_key 		= (is_key_pressed(KEY_W)) ? weight_fwd_key + dt 		: 0.0;
	weight_left_key 	= (is_key_pressed(KEY_A)) ? weight_left_key + dt 	: 0.0;
	weight_bwd_key 		= (is_key_pressed(KEY_S)) ? weight_bwd_key + dt 		: 0.0;
	weight_right_key 	= (is_key_pressed(KEY_D)) ? weight_right_key + dt 	: 0.0;
#endif
	
	/* saturate WASD keys weights between 0 and 1 */
	weight_fwd_key 		= max(min(weight_fwd_key, 	1.0), 0.0);
	weight_left_key 	= max(min(weight_left_key, 	1.0), 0.0);
	weight_bwd_key 		= max(min(weight_bwd_key, 	1.0), 0.0);
	weight_right_key 	= max(min(weight_right_key, 1.0), 0.0);
}


void update_mouse_position_gimbal(float sample_period, float *cmdYawGimbal, float *cmdPitchGimbal) {
	
	clock_ms_mouse_gimbal = clock_ms;
	float delta_t = clock_ms_mouse_gimbal - clock_ms_mouse_gimbal_prev;
	
	mouse_speed_x = ((float) get_remote_control_point()->mouse.x) * 0.001;	// 0.001 = millimeter/meter
	mouse_speed_y = ((float) get_remote_control_point()->mouse.y) * 0.001;
	
	saturate(&mouse_speed_x, MAX_MOUSE_SPEED_X);
	saturate(&mouse_speed_y, MAX_MOUSE_SPEED_Y);
	
	*cmdYawGimbal -= ((mouse_speed_x_prev + (mouse_speed_x-mouse_speed_x_prev)/2) * sample_period)/mouse_meters_yaw_rad_ratio;
	*cmdPitchGimbal -= ((mouse_speed_y_prev + (mouse_speed_y-mouse_speed_y_prev)/2) * sample_period)/mouse_meters_pitch_rad_ratio;
	
	mouse_speed_x_prev = mouse_speed_x;
	mouse_speed_y_prev = mouse_speed_y;
	
	clock_ms_mouse_gimbal_prev = clock_ms_mouse_gimbal;
}


void check_keyboard_commands_shoot_motors(uint8_t shooting_control_mode) {
	
	if (shooting_control_mode == NO_SHOOT_MODE)		// check, just to be sure
		return;
	
	// set reference speed for shoot motors
	if (is_key_raising_edge(KEY_Q) && !shoot_motors_speed_increased) {
		ref_shoot_motors_speed_radsec += (is_key_pressed(KEY_CTRL)) ? -50 : 50;
		ref_shoot_motors_speed_radsec = min(max(ref_shoot_motors_speed_radsec, 0), 800);	// mechanical speed limit of shoot motors is around 920 rad/s
		shoot_motors_speed_increased = 1;
	}
	else if (is_key_falling_edge(KEY_Q)) {
		shoot_motors_speed_increased = 0;
	}
}

void check_keyboard_commands_gimbal(uint8_t robot_control_mode) {
	
	if (robot_control_mode == ERROR_STOP_MODE)		// check, just to be sure
		return;
	
	// set gain for gimbal's pitch
	if (is_key_raising_edge(KEY_F) && !gain_LQR_pitch_increased) {
		if (initial_gimbal_pitch_gain == 0.0) {
			initial_gimbal_pitch_gain = gain_LQR_control_signal_gimbal_pitch;
		}
		gain_LQR_control_signal_gimbal_pitch += (is_key_pressed(KEY_CTRL)) ? -50 : 50;
		gain_LQR_control_signal_gimbal_pitch = min(max(gain_LQR_control_signal_gimbal_pitch, initial_gimbal_pitch_gain*0.5), initial_gimbal_pitch_gain*1.5);
		gain_LQR_pitch_increased = 1;
	}
	else if (is_key_falling_edge(KEY_F)) {
		gain_LQR_pitch_increased = 0;
	}
	
	// set gain for gimbal's yaw
	if (is_key_raising_edge(KEY_G) && !gain_LQR_yaw_increased) {
		if (initial_gimbal_yaw_gain == 0.0) {
			initial_gimbal_yaw_gain = gain_LQR_control_signal_gimbal_yaw;
		}
		gain_LQR_control_signal_gimbal_yaw += (is_key_pressed(KEY_CTRL)) ? -50 : 50;
		gain_LQR_control_signal_gimbal_yaw = min(max(gain_LQR_control_signal_gimbal_yaw, initial_gimbal_yaw_gain*0.5), initial_gimbal_yaw_gain*1.5);
		gain_LQR_yaw_increased = 1;
	}
	else if (is_key_falling_edge(KEY_G)) {
		gain_LQR_yaw_increased = 0;
	}
	
	// set mouse sensibility for gimbal's pitch
	if (is_key_raising_edge(KEY_V) && !mouse_sensibility_pitch_increased) {
		mouse_meters_pitch_rad_ratio += (is_key_pressed(KEY_CTRL)) ? 0.001 : -0.001;
		mouse_meters_pitch_rad_ratio = min(max(mouse_meters_pitch_rad_ratio, 0.004), 0.03);
		mouse_sensibility_pitch_increased = 1;
	}
	else if (is_key_falling_edge(KEY_V)) {
		mouse_sensibility_pitch_increased = 0;
	}
	
	// set mouse sensibility for gimbal's yaw
	if (is_key_raising_edge(KEY_B) && !mouse_sensibility_yaw_increased) {
		mouse_meters_yaw_rad_ratio += (is_key_pressed(KEY_CTRL)) ? 0.001 : -0.001;
		mouse_meters_yaw_rad_ratio = min(max(mouse_meters_yaw_rad_ratio, 0.004), 0.03);
		mouse_sensibility_yaw_increased = 1;
	}
	else if (is_key_falling_edge(KEY_B)) {
		mouse_sensibility_yaw_increased = 0;
	}
}


/* TODO refactor the following code
void check_keyboard_commands_balancing(uint8_t robot_control_mode) {
	
	if (robot_control_mode == ERROR_STOP_MODE)		// check, just to be sure
		return;
	
	// TODO delete this brake stuff (br needs to brake automatically when there're no commands from rc or keyboard)
	// brake BR movement
	if (is_key_raising_edge(KEY_C) && !should_brake_lin_vel &&
			((BR_chassis.state_estim[1] > 0 && BR_chassis.state_estim[3] > 0) || (BR_chassis.state_estim[1] < 0 && BR_chassis.state_estim[3] < 0)))
	{
		should_brake_lin_vel = 1;
		direction_BR_wheels_to_break = (BR_chassis.state_estim[1] > 0) ? 1 : -1;
	}
	else if (is_key_falling_edge(KEY_C))
	{
		should_brake_lin_vel = 0;
//		BR_movement_braked_successfully = 0;
	}
	
	if (should_brake_lin_vel &&
			((direction_BR_wheels_to_break == 1 && BR_chassis.state_estim[1] < 0 && BR_chassis.state_estim[3] < 0) ||
			(direction_BR_wheels_to_break == -1 && BR_chassis.state_estim[1] > 0 && BR_chassis.state_estim[3] > 0))) {
		
		should_brake_lin_vel = 0;
		direction_BR_wheels_to_break = 0;
	}
			
	
	// rotate BR chassis contiguously around itself
	if (is_key_raising_edge(KEY_E) && !should_rotate_chassis)
	{
		should_rotate_chassis = 1;
	}
	else if (is_key_falling_edge(KEY_E))
	{
		should_rotate_chassis = 0;
	}
	
	
	// modify balance theta value of BR
	if (is_key_raising_edge(KEY_Z) && !theta_balance_value_increased) {
		
		// HERE MODIFY theta_balance_offset_chassis_follow_gimbal (not theta_balance_offset) (done)
		theta_balance_offset_chassis_follow_gimbal += (is_key_pressed(KEY_CTRL)) ? 2*pi/180 : - 2*pi/180;
		theta_balance_offset_chassis_follow_gimbal = min(max(theta_balance_offset_chassis_follow_gimbal, - 15*pi/180), 15*pi/180);
		theta_balance_value_increased = 1;
	}
	else if (is_key_falling_edge(KEY_Z)) {
		theta_balance_value_increased = 0;
	}
}
*/



void gimbal_model_init(gimbal_model_t *model) {
	
	/* state estimations, references and errors */
	for (int i = 0; i < 4; i++) {
		model->state_estim_prev[i] = 0;
		model->ref_prev[i] = 0;
		model->error[i] = 0;
		model->error_prev[i] = 0;
	}
	
	for (int i = 0; i < 2; i++) {
		model->integral_error[i] = 0;		// yaw and pitch angles integral error
	}
	
}


void sentry_chassis_model_init(sentry_chassis_model_t *model) {
	
	model->wheel_radius = 0.0;	// TODO substitute with real radius
	model->integral_error = 0;
	model->lin_pos_on_track_prev = 0;
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


void zpk2tf(uint8_t m, uint8_t n, float complex *z, float complex *p, float k, float *num, float *den) {

    // compute the tf numerator
    roots2coeff(m, z, num);

    // compute the tf denominator
    roots2coeff(n, p, den);

    // multiply the tf numerator by the tf gain
    for (int i = 0; i <= m; i++)
        num[i] *= k;
}


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


void fir_init(struct fir *fir, uint32_t len, float *coeff) {
	
	fir->len = len;									// set filter's length
	
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


