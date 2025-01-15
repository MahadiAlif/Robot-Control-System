#include "shoot_rev_control.h"
#include "control_util.h"
#include "shooting_task.h"
#include "referee.h"
#include "robot_config.h"
#include "referee_alg.h"


/**************************************************************
Notes about the Shoot and Rev:
- For the shooter of the sentry robot:
	0 -> left motor of top shooter, 1 -> right motor of top shooter, 2 -> left motor of bottom shooter, 3 -> right motor of bottom shooter
	For the shooter of stardard and balancing robot it's the same, but there are only 0 and 1 (i.e. there's only one shooter)
**************************************************************/

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/*** model ***/
shoot_motors_model_t shoot_motors;		// model of the left and right shoot motor
rev_motor_model_t rev_motor;					// model of the rev motor

/*** control ***/
float T_shoot = 0.001;
float T_rev = 0.001;
float shoot_P_gain[4];		// proportional gain of the shoot motor (for PID)
float shoot_I_gain[4];		// integral gain of the shoot motor (for PID)
float rev_LQR_K[2];				// LQR gain of the standard robot's rev motor
float rev_I_gain[2];			// integral gain for error w.r.t. rev motor's position
float ref_shoot_motors_speed_radsec = 0;	// needs to be set by the keyboard (this is just the initial value)
float initial_shoot_speed_when_restart = 0;	// initial speed of shoot motors when we activate either SINGLE_SHOOT_MODE or MULTIPLE_SHOOT_MODE
float saturation_control_signal_shoot_L;		// left shoot motor control signal saturation value
float saturation_control_signal_shoot_R;		// right shoot motor control signal saturation value
int rev_rotation_mode = 0;
int enable_shoot_one_bullet = 0;
float error_threshold_to_shoot_new_sigle_bullet = 10*pi/180;		// 5
float error_threshold_to_shoot_new_multiple_bullet = 30*pi/180;	// 20
uint32_t start_clock_to_prevent_rev_unlock_indefinitely = 0;		// starting time of the counter that makes sure that, sooner or later, the rev motor will terminate the "unlock rev motor" mode
float shooting_speed_to_shoot_motors_speed_coeff = 1;

/*** gains to be tuned ***/
// shoot (left and right) and rev motors gains
float gain_control_signal_shoot_L = 60;
float gain_control_signal_shoot_R = 60;
float gain_LQR_control_signal_rev = 5000;	// 7000 3000

/*** rev motor integral values ***/
float gain_integral_rev_error = 30;
float range_where_trigger_rev_integral = 25*pi/180;

/*** flags ***/
int need_to_initialize_shoot_motor = 1;
int need_to_initialize_rev_motor = 1;
int allowed_to_shoot_single_bullet = 0;		// 1 if we want to shoot and we actually can
int want_to_shoot_single_bullet = 0;			// 1 if we want to shoot
int need_to_unlock_rev_motor = 0;			// used to unlock the rev motor when bullets get stuck
int need_to_set_unlock_rev_motor_ref = 0;
static int should_spin_shoot_wheels;


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void shoot_rev_control_loop(uint8_t shooting_control_mode, uint8_t shooting_control_mode_prev) {
	
	// check commands from keyboard to shoot motors
	check_keyboard_commands_shoot_motors(shooting_control_mode);
	
	// shoot motors control
	shoot_control(shooting_control_mode, shooting_control_mode_prev);
	
	// rev motor control
	rev_control(shooting_control_mode, shooting_control_mode_prev);
	
	if (!IS_SHOOT_ENABLED) {
		shoot_motors.control_signals[0] = 0;
		shoot_motors.control_signals[1] = 0;
	}
	if (!IS_REV_ENABLED) {
		rev_motor.control_signal = 0;
	}
		
	// [output] send the control signals
	CAN_cmd_shoot((int16_t) shoot_motors.control_signals[0], (int16_t) shoot_motors.control_signals[1], (int16_t) rev_motor.control_signal);
	
}
void shoot_control(uint8_t shooting_control_mode, uint8_t shooting_control_mode_prev) {
	
	if (need_to_initialize_shoot_motor)
	{	
		shoot_control_init();
		need_to_initialize_shoot_motor = 0;
	}
	
	// decide if shoot wheels should spin or not
	if (abs((int)rc_up_left_wheel) >= 5)
		should_spin_shoot_wheels = 1;
	else
		should_spin_shoot_wheels = 0;
		
	
	
	// measure the current speed of the shoot motors
	shoot_motors.y[0] = (get_shoot_motor_measures(0))->ang_vel_radsec;
	shoot_motors.y[1] = (get_shoot_motor_measures(1))->ang_vel_radsec;
	
	// compute the state estimations
	shoot_motors.speed_radsec[0] = shoot_motors.y[0];
	shoot_motors.speed_radsec[1] = shoot_motors.y[1];
	
	// set the reference signals
	if ((shooting_control_mode_prev == NO_SHOOT_MODE && shooting_control_mode != NO_SHOOT_MODE) ||
			initial_shoot_speed_when_restart != (get_robot_state().shooter_id1_17mm_speed_limit) * shooting_speed_to_shoot_motors_speed_coeff * SECURITY_PERCENTAGE_SHOOT_SPEED ||
			is_key_pressed(KEY_X)) {
				
		// compute initial speed of shoot motors (depending on robot's level)
		initial_shoot_speed_when_restart = (get_robot_state().shooter_id1_17mm_speed_limit) * shooting_speed_to_shoot_motors_speed_coeff * SECURITY_PERCENTAGE_SHOOT_SPEED;
		ref_shoot_motors_speed_radsec = initial_shoot_speed_when_restart;
		ref_shoot_motors_speed_radsec = 500;
	}
			
	if (USE_REFEREE_SYS_SHOOTING_SPEED)
		check_shoot_speed();
	
	// in order to shoot, we need to hold the right-key of mouse
	if (/*1*/ get_remote_control_point()->mouse.press_r || should_spin_shoot_wheels) {
		shoot_motors.ref[0] = - ref_shoot_motors_speed_radsec;		// [ref_shoot_motors_speed_radsec] = rad/s
		shoot_motors.ref[1] = ref_shoot_motors_speed_radsec;
	}
	else {
		shoot_motors.ref[0] = 0;
		shoot_motors.ref[1] = 0;
	}
	
	// compute the control signals
	PI_controller(SHOOT_MOTORS);
//	LQR_controller(SHOOT_MOTORS);

	// overall gain of shoot motors
	shoot_motors.control_signals[0] *= gain_control_signal_shoot_L;
	shoot_motors.control_signals[1] *= gain_control_signal_shoot_R;
	
	// saturation of the control signals (in order to avoid possible damages)
	saturate(&shoot_motors.control_signals[0], MAX_SHOOT_CONTROL_SIGNAL_AMPLITUDE);
	saturate(&shoot_motors.control_signals[1], MAX_SHOOT_CONTROL_SIGNAL_AMPLITUDE);
	
	// set previous values for next iteration
	for (int i = 0; i < 4; i++) {
		shoot_motors.ref_prev[i] = shoot_motors.ref[i];
		shoot_motors.error_prev[i] = shoot_motors.error[i];
	}
	
}


void rev_control(uint8_t shooting_control_mode, uint8_t shooting_control_mode_prev) {
	
	if (need_to_initialize_rev_motor)
	{
		rev_control_init();
		need_to_initialize_rev_motor = 0;
	}
	
	// measure the current speed of the shoot motors
	rev_motor.y[0] = (get_rev_motor_measures())->contig_ang_pos_rad;
	rev_motor.y[1] = (get_rev_motor_measures())->ang_vel_radsec;
	
	// compute the state estimations
	rev_motor.state_estim[0] = rev_motor.y[0];
	rev_motor.state_estim[1] = rev_motor.y[1];

	// check if left-key of mouse is pressed (if yes then we wanna shoot, but we need to see if the referee system allows us)
	
	/*
	With commands from PC mouse (to be modified maybe):
	(shooting_control_mode == SINGLE_SHOOT_MODE && (get_remote_control_point()->mouse.press_l) && !(get_old_remote_control_point()->mouse.press_l)) ||
	(shooting_control_mode == MULTIPLE_SHOOT_MODE && (get_remote_control_point()->mouse.press_l)) ||
	(shooting_control_mode == MULTIPLE_SHOOT_MODE)
	*/
	if ((shooting_control_mode_prev == NO_SHOOT_MODE && shooting_control_mode == SINGLE_SHOOT_MODE) ||
			(shooting_control_mode == MULTIPLE_SHOOT_MODE))
	{
		allowed_to_shoot_single_bullet = 1;
		want_to_shoot_single_bullet = 1;
	}
	

	// if we want to shoot, then check all possible conditions that could prevent us from shooting
	if (want_to_shoot_single_bullet) {
		
		// check if we already actually shooted the previous bullet
		if (((shooting_control_mode == SINGLE_SHOOT_MODE) && (fabs(rev_motor.error[0]) > error_threshold_to_shoot_new_sigle_bullet)) ||
				((shooting_control_mode == MULTIPLE_SHOOT_MODE) && (fabs(rev_motor.error[0]) > error_threshold_to_shoot_new_multiple_bullet)))
			allowed_to_shoot_single_bullet = 0;	// deny to shoot the new bullet
		
		/* see if we can actually shoot (depending on the current "barrel_heat_estimation" (= real_barrel_heat + pre_shoot_heat)),
			 and if we allow shooting then update "barrel_heat_estimation" */
		if (USE_REFEREE_SYS_SHOOTING_FREQUENCY)
			check_barrel_heat_shooting();
		
		/*NOTE: the check of the barrel heat should always be the last check that we do to decide if to shoot or not (because we update "barrel_heat.pre_shoot_heat")*/
	}
	
	if (!need_to_unlock_rev_motor)
	{	
		rev_motor.ref[0] += (allowed_to_shoot_single_bullet) ? -pi/4 : 0;
		rev_motor.ref[1] = 0;
	}
	else if (need_to_set_unlock_rev_motor_ref)
	{
		rev_motor.ref[0] = rev_motor.state_estim[0] + pi/4;
		rev_motor.ref[1] = 0;
		need_to_set_unlock_rev_motor_ref = 0;
	}
	
	
	// compute the control signals
	LQR_controller(REV_MOTOR);
	
	// saturation of the control signals (in order to avoid possible damages)
	saturate(&rev_motor.control_signal, MAX_REV_CONTROL_SIGNAL_AMPLITUDE);
	
	// check if the rev motor is stuck because of some bullet in the shooting system (if yes, then need to release)
	if (!need_to_unlock_rev_motor && want_to_shoot_single_bullet && !allowed_to_shoot_single_bullet &&
			fabs(rev_motor.error[0] - rev_motor.error_prev[0]) < 5*pi/180 && fabs(rev_motor.state_estim[1]) <= 2*pi/180) {
	
		need_to_unlock_rev_motor = 1;
		need_to_set_unlock_rev_motor_ref = 1;
		start_clock_to_prevent_rev_unlock_indefinitely = clock_ms;
	}
	else if (need_to_unlock_rev_motor && (fabs(rev_motor.error[0]) <= 5*pi/180 || (clock_ms - start_clock_to_prevent_rev_unlock_indefinitely) >= (uint32_t) 1/T_gimbal)) {
		
		rev_motor.ref[0] = rev_motor.state_estim[0];		// set new initial reference after "unlock rev motor" routine
		need_to_unlock_rev_motor = 0;
	}
	
	// increase the counter that makes sure that, sooner or later, the rev motor will terminate the "unlock rev motor" mode
//	if (need_to_unlock_rev_motor)
//		counter_to_prevent_rev_unlock_indefinitely += 1;
	
	// set previous values for next iteration
	for (int i = 0; i < 2; i++) {
		rev_motor.ref_prev[i] = rev_motor.ref[i];
		rev_motor.error_prev[i] = rev_motor.error[i];
	}
	
	// reset the decision regarding if to shoot or not
	allowed_to_shoot_single_bullet = 0;
	want_to_shoot_single_bullet = 0;
}


void shoot_control_init(void) {
	
	shoot_model_init(&shoot_motors);			// initialization of the model
	PI_gain_init(SHOOT_MOTORS);
//	LQR_gain_init(STANDARD_GIMBAL);						// initialization of the LQR gain for the gimbal

}

void rev_control_init(void) {
	
	rev_model_init(&rev_motor);
	LQR_gain_init(REV_MOTOR);
}



