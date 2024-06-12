#include <stdlib.h>
#include <math.h>
#include "math_util.h"
#include "control_alg.h"
#include "gimbal_control.h"
#include "br_chassis_control.h"
#include "std_chassis_control.h"
#include "shoot_rev_control.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "robot_config.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

// exponential gain coefficients
float k;
float tau;
float offset;
float coeff;

// gimbal mantain position coefficients
float x2, x3, x4, xm;
float offset_zero;
float eta;
float k_coeff;




/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

void LQR_gain_init(int id_robot_part) {
	
	if (id_robot_part == REV_MOTOR) {
		
		rev_LQR_K[0] = 10;
		rev_LQR_K[1] = 0.9373425;	 // 0.9373425 = 0.5*M3508_gearbox_ratio/M2006_gearbox_ratio;
	}
	else
		return;
}


void LQR_controller(int id_robot_part) {
	
	/*NOTE: w.r.t. the classic LQR, in the gimbal and rev motor control it is added an integrative part (hence it is like a PID) */
	
	if (id_robot_part == STANDARD_GIMBAL) {
		
//		// compute error
//		for (int i = 0; i < 4; i++) {
//			standard_gimbal.error[i] = standard_gimbal.ref[i] - standard_gimbal.state_estim[i];
//		}
//		
//		if (standard_gimbal.error[0] * standard_gimbal.error_prev[0] < 0)
//			standard_gimbal.integral_error[0] = 0;
//		else
//			standard_gimbal.integral_error[0] += standard_gimbal.error[0] * T_gimbal;
//		
//		standard_gimbal.integral_error[1] += standard_gimbal.error[1] * T_gimbal;
//		
//		// saturate yaw and pitch integral error
//		saturate(&standard_gimbal.integral_error[0], sat_ei_yaw);
//		saturate(&standard_gimbal.integral_error[1], sat_ei_pitch);
		
		
	}
	else if (id_robot_part == REV_MOTOR) {
		
		float cmd = 0;
		
		// compute error
		for (int i = 0; i < 2; i++) {
			rev_motor.error[i] = rev_motor.ref[i] - rev_motor.state_estim[i];
		}
		
		// compute rev motor's position integral error
		if (rev_motor.error[0] * rev_motor.error_prev[0] < 0 ||
				fabs(rev_motor.error[0]) > range_where_trigger_rev_integral ||
				rev_motor.ref[0] != rev_motor.ref_prev[0])
		{
			rev_motor.integral_error[0] = 0;
		}
		else
			rev_motor.integral_error[0] += rev_motor.error[0] * T_rev;
		
		// saturate rev motor's position integral error
		saturate(&rev_motor.integral_error[0], MAX_REV_INTEGRATIVE_CONTROL_PART_AMPLITUDE);
		
		// compute control signals coming from LQR (without integral error part)
		for (int i = 0; i < 2; i++) {
			cmd += rev_LQR_K[i] * rev_motor.error[i];
		}
		
		// rev position integral gain
		rev_I_gain[0] = (fabs(rev_motor.error[0]) <= range_where_trigger_rev_integral) ? gain_integral_rev_error : 0;
		
		// add integral error part
		cmd += rev_I_gain[0] * rev_motor.integral_error[0];
		
		// compute control signals coming from LQR (considering the integral error part too)
		rev_motor.control_signal = gain_LQR_control_signal_rev * cmd;
	}
	else
		return;
}


void PI_gain_init(int id_robot_part) {
	
	if (id_robot_part == SHOOT_MOTORS) {
		
		shoot_P_gain[0] = 4;
		shoot_P_gain[1] = 4;
		shoot_P_gain[2] = 2;
		shoot_P_gain[3] = 2;
		shoot_I_gain[0] = 0.5;
		shoot_I_gain[1] = 0.5;
		shoot_I_gain[2] = 1;
		shoot_I_gain[3] = 1;
	}
	else
		return;
}


void PI_controller(int id_robot_part) {
	
	if (id_robot_part == SHOOT_MOTORS) {
		
		for (int i = 0; i < 2; i++) {
			
			// update error
			shoot_motors.error[i] = shoot_motors.ref[i] - shoot_motors.speed_radsec[i];
			
			// update integral error
			if (shoot_motors.ref[i] != shoot_motors.ref_prev[i] /* <= 0*/)
				shoot_motors.integral_error[i] = 0;
			else
				shoot_motors.integral_error[i] += shoot_motors.error[i] * T_shoot;
			
			// compute control signals
			shoot_motors.control_signals[i] = (shoot_P_gain[i] * shoot_motors.error[i]) +
																				(shoot_I_gain[i] * shoot_motors.integral_error[i]);
		}
	}
	else
		return;
}


/*
NOTES:
- the function returns a gain that is exponentially proportional to the error (i.e. error w.r.t. references)
- the exponential gain curve is such that it passes through the points (x1,y1), (x2,y2), (x3,y3)
- there must be "0 <= x1 < x2 < x3" and "y1 > y2 > y3 >= 1"
*/
float exp_error_gain(float error, float x1, float x2, float x3, float y1, float y2, float y3) {
	
	if (fabs(error) <= x3)
	{
		k = y1 - 1;
		tau = log((y2-1)/k)/x2;
		offset = k*exp(tau*x3);
		coeff = y1/(k - offset + 1);
		
		return coeff*(k*exp(tau*fabs(error)) - offset + 1);
	}
	else {
		return 1;
	}
}


void PID_init_(struct PID_ *pid, float Kp, float Ki, float Kd, float dt) {
	
	/* initialize Kp, Ki, Kd, N and dt */
	pid->Kp 			= Kp;
	pid->Ki 			= Ki;
	pid->Kd 			= Kd;
	pid->dt 			= dt;
	pid->up_prev 	= 0;
	pid->ui_prev 	= 0;
	pid->ud_prev 	= 0;
}


void LQR_init(struct LQR *lqr, float *K, float *Ki, uint8_t m, uint8_t n) {
	
	int is_K_given		= 1;
	int is_Ki_given 	= 1;
	
	if (K == NULL)
		is_K_given = 0;			// if K is not given, fill it with zeros
	if (Ki == NULL)
		is_Ki_given = 0;		// if K is not given, fill it with zeros
	
	/* set the value of each K and Ki entry */
	for (int row = 0; row < m; row++) {
		for (int col = 0; col < n; col++) {

			lqr->K[row*n + col] 	= (is_K_given) ? 	K[row*n + col] 	: 0;		// initialize K matrix gain
			
			lqr->Ki[row*n + col] 	= (is_Ki_given) ? Ki[row*n + col] : 0;		// initialize Ki matrix gain
		}
	}
}


float PID_control(struct PID_ *pid, float ep, float ei, float ed, float sat_up, float sat_ui, float sat_ud, struct fir *fir_up, struct fir *fir_ui, struct fir *fir_ud) {
	
	float Kp 				= pid->Kp;
	float Ki 				= pid->Ki;
	float Kd 				= pid->Kd;
	
	/* compute Proportional, Integral and Derivative components */
	float up = Kp * ep;
	float ui = Ki * ei;
	float ud = Kd * ed;
	
	// apply the low-pass filter to the Proportional, Integrative and Derivative part
	if (fir_up != NULL)
		up = fir_filter(fir_up, up);
	if (fir_ui != NULL)
		ui = fir_filter(fir_ui, ui);
	if (fir_ud != NULL)
		ud = fir_filter(fir_ud, ud);
	
	// saturate Proportional, Integral and Derivative components
	if (sat_up != UNDEF)
		saturate(&up, sat_up);
	if (sat_ui != UNDEF)
		saturate(&ui, sat_ui);
	if (sat_ud != UNDEF)
		saturate(&ud, sat_ud);
	
	// save previous control signal values
	pid->up_prev = up;
	pid->ui_prev = ui;
	pid->ud_prev = ud;
	
	// return overall control signal
	return up + ui + ud;
}


void LQR_control(struct robot *robot, struct LQR *lqr) {
	
	uint8_t m = robot->m;		// number of robot inputs
	uint8_t n = robot->n;		// number of robot states
	
	// LQR control signals derived from matrices K and Ki
	float u_K[m];
	float u_Ki[m];
	
	mat_mult(u_K, lqr->K, robot->error, m, n, 1);
	mat_mult(u_Ki, lqr->Ki, robot->integr_error, m, n, 1);
	
	for (int i = 0; i < m; i++)
		robot->u[i] = u_K[i] + u_Ki[i];		// sum LQR control signals derived from K and Ki
	
}


float tf_resp(struct tf *tf, float u) {
	
	// reset the tf response to u input
	float y = 0;

	// shift backward u[k] and y[k] values of one time unit (for all k) 
	for (int i = tf->n; i > 0; i--) {
		tf->u[i] = tf->u[i-1];
		tf->y[i] = tf->y[i-1];
  }
  tf->u[0] = u;		// new input
	
	/*
	*		compute new output y[k] based on:
	*		1. new input u[k]
	*		2. previous inputs u[k-i] (i > 0)
	*		3. previous outputs y[k-i] (i > 0)
	*/
	for (int i = 0; i <= tf->n; i++) {
		y += tf->num[i] * tf->u[i];
    if (i > 0)
			y -= tf->den[i] * tf->y[i];
  }
	y = y / tf->den[0];

	tf->y[0] = y;		// new output

  return y;
}


void ref_error(struct robot *robot, float dt) {
	
	for (int state = 0; state < robot->n; state++) {
		ref_error_state(robot, state, dt);
	}
}


void ref_error_state(struct robot *robot, int state, float dt) {
	
	robot->error[state] = robot->ref[state] - robot->x[state];													// error w.r.t. reference
	robot->integr_error[state] += robot->error[state] * dt;															// integral error w.r.t. reference
	robot->deriv_error[state] = (robot->error[state] - robot->error_prev[state]) / dt;	// integral error w.r.t. reference
}









