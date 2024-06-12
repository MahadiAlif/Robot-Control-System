#include "shoot_rev_control.h"
#include "control_util.h"
#include "shooting_task.h"
#include "referee.h"
#include "robot_config.h"
#include "referee_alg.h"
#include "gimbal_task.h"


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
float T_shoot = 0.002;
float T_rev = 0.002;
float shoot_P_gain[4];		// proportional gain of the shoot motor (for PID)
float shoot_I_gain[4];		// integral gain of the shoot motor (for PID)
float rev_LQR_K[2];				// LQR gain of the standard robot's rev motor
float rev_I_gain[2];			// integral gain for error w.r.t. rev motor's position
float ref_shoot_motors_speed_radsec = 0;	// needs to be set by the keyboard (this is just the initial value)
float initial_shoot_speed_when_restart = 500;	// initial speed of shoot motors when we activate either SINGLE_SHOOT_MODE or MULTIPLE_SHOOT_MODE
float saturation_control_signal_shoot_L;		// left shoot motor control signal saturation value
float saturation_control_signal_shoot_R;		// right shoot motor control signal saturation value
int rev_rotation_mode = 0;
int enable_shoot_one_bullet = 0;
float error_threshold_to_shoot_new_sigle_bullet = 10*pi/180;		// 5
float error_threshold_to_shoot_new_multiple_bullet = 10*pi/180;	// 10
uint32_t start_clock_to_prevent_rev_unlock_indefinitely = 0;		// starting time of the counter that makes sure that, sooner or later, the rev motor will terminate the "unlock rev motor" mode
float shooting_speed_to_shoot_motors_speed_coeff = 1;
uint32_t clock_tick_ms_spinning_wheel = 0;
//float rev_pos_new_bullet_multiple_shoot;								// REV motor position at which a new bullet has to be shot in "multiple shoot" mode		[rad]
float time_single_bullet_multiple_shoot = 0.05;						// time that it should take for each bullet to be shot in "multiple shoot" mode		[s]
float time_single_bullet_cv_multiple_shoot = 1/EPSILON;		// time that it should take for each bullet to be shot in "multiple shoot" mode when CV is active		[s]
float tick_ms_new_bullet_multiple_shoot;									// clock time at which a new bullet starts to be shot in "multiple shoot" mode		[ms]
//float rev_pos_new_bullet_triple_shoot;									// REV motor position at which a new bullet has to be shot in "triple shoot" mode		[rad]
float time_single_bullet_triple_shoot = 0.15;							// time that it should take for each bullet to be shot in "triple shoot" mode		[s]
float tick_ms_new_bullet_triple_shoot;										// clock time at which a new bullet starts to be shot in "triple shoot" mode		[ms]
//float rev_pos_new_bullet_single_shoot;									// REV motor position at which a new bullet has to be shot in "single shoot" mode		[rad]
float time_single_bullet_single_shoot = 0.05;							// time that it should take for each bullet to be shot in "single shoot" mode		[s]
float tick_ms_new_bullet_single_shoot;										// clock time at which a new bXZullet starts to be shot in "single shoot" mode		[ms]
float rev_pos_new_bullet_shoot;

float cv_shoot_freq = 0.0;													// shooting frequency for sentry robot (determined by how far is the enemy robot detected by the CV)

/*** gains to be tuned ***/
// shoot (left and right) and rev motors gains
float gain_control_signal_shoot_L = 60;
float gain_control_signal_shoot_R = 60;
#if IS_STD
float gain_LQR_control_signal_rev = 3000;
#elif IS_BR2
float gain_LQR_control_signal_rev = 2600;
#elif IS_SENTRY
float gain_LQR_control_signal_rev = 2600;
#endif

/*** rev motor integral values ***/
float gain_integral_rev_error = 30; 
float range_where_trigger_rev_integral = 50*pi/180;		// 25

/*** flags ***/
int need_to_initialize_shoot_motor = 1;
int need_to_initialize_rev_motor = 1;
int allowed_to_shoot_single_bullet = 0;		// 1 if we want to shoot and we actually can
int want_to_shoot_single_bullet = 0;			// 1 if we want to shoot
int need_to_unlock_rev_motor = 0;			// used to unlock the rev motor when bullets get stuck
int need_to_set_unlock_rev_motor_ref = 0;
static int should_spin_shoot_wheels;
uint8_t is_shooting_wheel_on = 0;
uint8_t is_first_iter = 1;
uint8_t unlock_flag = 1;

struct PID_ pid_rev;
float Kp_rev = 4.0;
float Ki_rev = 2.0;
float Kd_rev = 0.15;

/*** time variables ***/
float time_unlock_activation;

// TODO write better
float tmp_shoot = 480; // 460


float ref_rev_offset;


uint8_t active_shooting = NO_SHOOT_MODE;
uint8_t active_shooting_prev = NO_SHOOT_MODE;

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
	if (!IS_REV_ENABLED || switch_is_down(rc_right_switch)) {
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
//	if ((shooting_control_mode_prev == NO_SHOOT_MODE && shooting_control_mode != NO_SHOOT_MODE) ||
//			initial_shoot_speed_when_restart != (get_robot_state().shooter_id1_17mm_speed_limit) * shooting_speed_to_shoot_motors_speed_coeff * SECURITY_PERCENTAGE_SHOOT_SPEED ||
//			is_key_pressed(KEY_X)) {
//				
//		// compute initial speed of shoot motors (depending on robot's level)
//		initial_shoot_speed_when_restart = (get_robot_state().shooter_id1_17mm_speed_limit) * shooting_speed_to_shoot_motors_speed_coeff * SECURITY_PERCENTAGE_SHOOT_SPEED;
//		initial_shoot_speed_when_restart = 500;
//		ref_shoot_motors_speed_radsec = initial_shoot_speed_when_restart;
//	}
//	
//	if (USE_REFEREE_SYS_SHOOT_SPEED)
//		check_shoot_speed();
	
	ref_shoot_motors_speed_radsec = tmp_shoot;
	
// remote controller shooting wheel activation
	if (switch_is_up(rc_right_switch)){
		
		if(should_spin_shoot_wheels) {
		
		shoot_motors.ref[0] = - ref_shoot_motors_speed_radsec;		// [ref_shoot_motors_speed_radsec] = rad/s
		shoot_motors.ref[1] = ref_shoot_motors_speed_radsec;
		clock_tick_ms_spinning_wheel = HAL_GetTick();
		}
		else if (HAL_GetTick() - clock_tick_ms_spinning_wheel > 1*1e3) {
		shoot_motors.ref[0] = 0;
		shoot_motors.ref[1] = 0;
		}
	}
	// key_board shooting wheel activation
	else if (switch_is_mid(rc_right_switch)){
		
		if((is_key_raising_edge(KEY_G) && !is_shooting_wheel_on) || is_first_iter || !switch_is_mid(rc_right_switch_prev)){
		
		is_shooting_wheel_on = 1;
		shoot_motors.ref[0] = - ref_shoot_motors_speed_radsec;		// [ref_shoot_motors_speed_radsec] = rad/s
		shoot_motors.ref[1] = ref_shoot_motors_speed_radsec;
		
		}		
	
		else if (is_shooting_wheel_on && is_key_raising_edge(KEY_G)){
		
		is_shooting_wheel_on = 0;
		shoot_motors.ref[0] = 0;
		shoot_motors.ref[1] = 0;
		}
		is_first_iter = 0;
	}
	else if (switch_is_down(rc_right_switch)) {
		
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
	
	// if the shooting wheels are almost-stopped and we don't want to shoot, then don't give any command at all (do NOT constantly force the shooting wheels to stay at 0 velocity)
	if (fabs(shoot_motors.ref[0]) <= 2*pi && fabs(shoot_motors.speed_radsec[0]) <= 2*pi)
		shoot_motors.control_signals[0] = 0.;
	if (fabs(shoot_motors.ref[1]) <= 2*pi && fabs(shoot_motors.speed_radsec[1]) <= 2*pi)
		shoot_motors.control_signals[1] = 0;
	
	
	// set previous values for next iteration
	for (int i = 0; i < 4; i++) {
		shoot_motors.ref_prev[i] = shoot_motors.ref[i];
		shoot_motors.error_prev[i] = shoot_motors.error[i];
	}
	
}


void rev_control(uint8_t shooting_control_mode, uint8_t shooting_control_mode_prev) {
	
	if (switch_is_down(rc_right_switch))
		return;
	
	if (need_to_initialize_rev_motor)
	{
		rev_control_init();
		
		PID_init_(&pid_rev, Kp_rev, Ki_rev, Kd_rev, 0.005);
		
		need_to_initialize_rev_motor = 0;
	}
	
	// measure the current speed of the shoot motors
	rev_motor.y[0] = (get_rev_motor_measures())->contig_ang_pos_rad;
	rev_motor.y[1] = (get_rev_motor_measures())->ang_vel_radsec;
	
	// compute the state estimations
	rev_motor.state_estim[0] = rev_motor.y[0];
	rev_motor.state_estim[1] = rev_motor.y[1];

	// check if we wanna shoot (but, below, we also need to see if the current shooting system status allows us)
	if ((shooting_control_mode_prev == NO_SHOOT_MODE && shooting_control_mode == SINGLE_SHOOT_MODE) ||
			(shooting_control_mode_prev == NO_SHOOT_MODE && shooting_control_mode == TRIPLE_SHOOT_MODE) ||
		  (shooting_control_mode == MULTIPLE_SHOOT_MODE))
	{
		allowed_to_shoot_single_bullet = 1;
		want_to_shoot_single_bullet = 1;
	}
	
	if (shooting_control_mode_prev == NO_SHOOT_MODE && shooting_control_mode == SINGLE_SHOOT_MODE)
		active_shooting = SINGLE_SHOOT_MODE;
	else if (active_shooting == SINGLE_SHOOT_MODE && ref_rev_offset <= -pi/4)
		active_shooting = NO_SHOOT_MODE;
	
	if (shooting_control_mode_prev == NO_SHOOT_MODE && shooting_control_mode == TRIPLE_SHOOT_MODE)
		active_shooting = TRIPLE_SHOOT_MODE;
	else if (active_shooting == TRIPLE_SHOOT_MODE && ref_rev_offset <= -4*pi/4)
		active_shooting = NO_SHOOT_MODE;
	

	// if we want to shoot, then check all possible conditions that could prevent us from shooting
	if (want_to_shoot_single_bullet) {
		
		// check if we already actually shooted the previous bullet
		if (((shooting_control_mode == SINGLE_SHOOT_MODE) && (fabs(rev_motor.error[0]) > error_threshold_to_shoot_new_sigle_bullet)) ||
				((shooting_control_mode == TRIPLE_SHOOT_MODE) && (fabs(rev_motor.error[0]) > error_threshold_to_shoot_new_multiple_bullet)) || 
				((shooting_control_mode == MULTIPLE_SHOOT_MODE) && (fabs(rev_motor.error[0]) > error_threshold_to_shoot_new_multiple_bullet)))
			allowed_to_shoot_single_bullet = 0;	// deny to shoot the new bullet
		
		if ((shooting_control_mode == MULTIPLE_SHOOT_MODE) && (ref_rev_offset > -pi/4))
			allowed_to_shoot_single_bullet = 0;	// deny to shoot the new bullet
		
		if (active_shooting != NO_SHOOT_MODE && active_shooting_prev != NO_SHOOT_MODE)
			allowed_to_shoot_single_bullet = 0;

#if IS_SENTRY
		float cv_yaw_error = asin(- cv_gimbal.x_cam / cv_gimbal.d_xyz);
		
		if (sentry_state != IDLE && sentry_state != MOVE_TO_SPOT && sentry_state != PATROL &&
				fabs(cv_yaw_error) <= 10*DEG_TO_RAD)
			allowed_to_shoot_single_bullet = 0;
#endif
		
		/* check that we didn't exceed the shooting heat of referee system; if yes, then do NOT shoots */
#if USE_REFEREE_SYS_SHOOT_HEAT
		check_barrel_heat_shooting(&allowed_to_shoot_single_bullet);
#endif
		
		/*NOTE: the check of the barrel heat should always be the last check that we do to decide if to shoot or not (because we update "barrel_heat.pre_shoot_heat")*/
	}

	
	// for the sentry, set the shooting frequency (depending on how far is the detected enemy)
#if IS_SENTRY && IS_SENTRY_AUTO_SHOOT_ENABLED
	if (switch_is_mid(rc_right_switch))						// TODO here we could add the condition "&& shooting_control_mode == MULTIPLE_SHOOT_MODE", but it's probably useless
		time_single_bullet_cv_multiple_shoot = 1/(cv_shoot_freq + EPSILON);
#endif
	
	
	if (!need_to_unlock_rev_motor)
	{
		//float ref_rev_offset;		// TODO re-enable this (and cancel the global one)
		
		/* compute the offset to be added to the current REV motor position reference (depending on the current shooting mode) */
		if (active_shooting == SINGLE_SHOOT_MODE) {
			
			// check if a new bullet is going to be shot in "triple shoot" mode
			if (allowed_to_shoot_single_bullet) {
				
				rev_pos_new_bullet_shoot = rev_motor.state_estim[0];
				tick_ms_new_bullet_single_shoot = HAL_GetTick();
			}
			ref_rev_offset = (-pi/4) * ((HAL_GetTick() - tick_ms_new_bullet_single_shoot) / (time_single_bullet_single_shoot*1e3));
			saturate(&ref_rev_offset, pi/4);		// cannot exceed the -3*pi/4 offset (which is equivalent to shooting one bullet)
			
			// shoot one bullet
			//ref_rev_offset = -pi/4;
		}
		else if (active_shooting == TRIPLE_SHOOT_MODE) {
			
			// check if a new bullet is going to be shot in "triple shoot" mode
			if (allowed_to_shoot_single_bullet) {
				
				rev_pos_new_bullet_shoot = rev_motor.state_estim[0];
				tick_ms_new_bullet_triple_shoot = HAL_GetTick();
			}
			ref_rev_offset = (-4*pi/4) * ((HAL_GetTick() - tick_ms_new_bullet_triple_shoot) / (time_single_bullet_triple_shoot*1e3));
			saturate(&ref_rev_offset, 4*pi/4);		// cannot exceed the -3*pi/4 offset (which is equivalent to shooting one bullet)
			
			
			// shoot three bullets
//			ref_rev_offset = -3*pi/4;
		}
		else if (shooting_control_mode == MULTIPLE_SHOOT_MODE) {
			
			// check if a new bullet is going to be shot in "multiple shoot" mode
			if (allowed_to_shoot_single_bullet || shooting_control_mode_prev != MULTIPLE_SHOOT_MODE) {
				
				rev_pos_new_bullet_shoot = rev_motor.state_estim[0];
				tick_ms_new_bullet_multiple_shoot = HAL_GetTick();
			}
			
#if IS_SENTRY && IS_SENTRY_AUTO_SHOOT_ENABLED
			float time_bullet = switch_is_mid(rc_right_switch) ? time_single_bullet_cv_multiple_shoot : time_single_bullet_multiple_shoot;
#else
			float time_bullet = time_single_bullet_multiple_shoot;
#endif
			ref_rev_offset = (-pi/4) * ((HAL_GetTick() - tick_ms_new_bullet_multiple_shoot) / (time_bullet*1e3));
			saturate(&ref_rev_offset, pi/4);		// cannot exceed the -pi/4 offset (which is equivalent to shooting one bullet)
		}
		else {
			
			ref_rev_offset = 0;
		}
		
		// add offset to current REV motor position reference (depending on the current shooting mode)
//		if (shooting_control_mode == MULTIPLE_SHOOT_MODE)
		rev_motor.ref[0] = rev_pos_new_bullet_shoot + ref_rev_offset;
//		else
//			rev_motor.ref[0] += ref_rev_offset;
		
		rev_motor.ref[1] = 0;
		
		
		// avoid backward movements of REV motor
		if (!need_to_unlock_rev_motor && rev_motor.ref[0] > rev_motor.ref_prev[0])
			rev_motor.ref[0] = rev_motor.ref_prev[0];
		
	}
	
	// rev motor unlock with vibration
	else
	{
		if(HAL_GetTick() - time_unlock_activation > 0.08e3 && unlock_flag ){
			unlock_flag = 0;
			rev_motor.ref[0] = rev_motor.state_estim[0] + 20*pi/180;
			rev_motor.ref[1] = 0;
			
		}
		else if(HAL_GetTick() - time_unlock_activation > 0.16e3){
			unlock_flag = 1;
			rev_motor.ref[0] = rev_motor.state_estim[0] - 20*pi/180;
			rev_motor.ref[1] = 0;			
			time_unlock_activation = HAL_GetTick();
		}
	}
	
	
	if (switch_is_down(rc_right_switch_prev) && !switch_is_down(rc_right_switch))
		rev_motor.ref[0] = rev_motor.state_estim[0];
	
	
	// compute the control signals
	LQR_controller(REV_MOTOR);
	
	
	rev_motor.control_signal = PID_control(&pid_rev,	rev_motor.error[0],	rev_motor.integral_error[0],	rev_motor.error[1],
																										UNDEF,							UNDEF,												UNDEF,
																										NULL,								NULL,													NULL);
	
	rev_motor.control_signal *= gain_LQR_control_signal_rev;
	
	
	// saturation of the control signals (in order to avoid possible damages)
	saturate(&rev_motor.control_signal, MAX_REV_CONTROL_SIGNAL_AMPLITUDE);
	
	// check if the rev motor is stuck because of some bullet in the shooting system (if yes, then need to release)
	if ((!need_to_unlock_rev_motor && !allowed_to_shoot_single_bullet && /*want_to_shoot_single_bullet &&*/
			(fabs(rev_motor.error[0]) > 10*pi/180) && (fabs(rev_motor.state_estim[1]) < 0.5*pi/180) && (fabs(rev_motor.error[0] - rev_motor.error_prev[0]) < 1*pi/180))
			|| (is_key_pressed(KEY_V))) {
		
		time_unlock_activation = HAL_GetTick();
		need_to_unlock_rev_motor = 1;
		need_to_set_unlock_rev_motor_ref = 1;
		start_clock_to_prevent_rev_unlock_indefinitely = clock_ms;
	}
	else if (need_to_unlock_rev_motor && (/*fabs(rev_motor.error[0]) <= 5*pi/180 ||*/ (clock_ms - start_clock_to_prevent_rev_unlock_indefinitely) >= (uint32_t) 1/dt_gimbal)) {
		
		rev_motor.ref[0] = rev_motor.state_estim[0];		// set new initial reference after "unlock rev motor" routine
		need_to_unlock_rev_motor = 0;
	}
	
	
	// set previous values for next iteration
	for (int i = 0; i < 2; i++) {
		rev_motor.ref_prev[i] = rev_motor.ref[i];
		rev_motor.error_prev[i] = rev_motor.error[i];
	}
	
	// reset the decision regarding if to shoot or not
	allowed_to_shoot_single_bullet = 0;
	want_to_shoot_single_bullet = 0;
	
	// store the shooting mode that was active in the previous iteration (for next iterations)
	active_shooting_prev = active_shooting;
	
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



