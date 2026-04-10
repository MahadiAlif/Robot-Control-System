#include <stdlib.h>
#include <math.h>
#include "math_util.h"
#include "control_alg.h"
#include "gimbal_control.h"
#include "br_chassis_control.h"
#include "std_chassis_control.h"
#include "shoot_rev_control.h"
#include "sentry_chassis_control.h"
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
	
	if (id_robot_part == STANDARD_GIMBAL) {
		
		if (IS_STD) {
			gimbal_yaw_LQR_K[0] = 7.0711;
			gimbal_yaw_LQR_K[1] = 10.4989*pi/180;		//5.4989
			gimbal_pitch_LQR_K[0] = 7.0711;
			gimbal_pitch_LQR_K[1] = 0.407266468; //5*pi/180;		//12.4828
		}
		else if (IS_BR1 || IS_BR2) {
			gimbal_yaw_LQR_K[0] = 7.0711;
			gimbal_yaw_LQR_K[1] = 10.4989*pi/180;
			gimbal_pitch_LQR_K[0] = 7.0711;
			gimbal_pitch_LQR_K[1] = 0; //0.407266468;
		}
	}
	else if (id_robot_part == REV_MOTOR) {
		
		rev_LQR_K[0] = 10;
		rev_LQR_K[1] = 0.9373425;	 // 0.9373425 = 0.5*M3508_gearbox_ratio/M2006_gearbox_ratio;
	}
	else
		return;
}


void LQR_controller(int id_robot_part) {
	
	/*NOTE: w.r.t. the classic LQR, in the gimbal and rev motor control it is added an integrative part (hence it is like a PID) */
	
	if (id_robot_part == STANDARD_GIMBAL) {
		
		float cmd_yaw = 0;
		float cmd_pitch = 0;
		
		// compute error
		for (int i = 0; i < 4; i++) {
			standard_gimbal.error[i] = standard_gimbal.ref[i] - standard_gimbal.state_estim[i];
		}
		
		// compute yaw and pitch integral error
//		if (standard_gimbal.error[0] * standard_gimbal.error_prev[0] < 0 ||
//				fabs(standard_gimbal.error[0]) > range_where_trigger_yaw_integral ||
//				standard_gimbal.ref[0] != standard_gimbal.ref_prev[0]/* ||
//				robot_control_mode != FIXED_GIMBAL_PC_MODE*/)
//		{
//			standard_gimbal.integral_error[0] = 0;
//		}
//		else
		
		if (standard_gimbal.error[0] * standard_gimbal.error_prev[0] < 0)
			standard_gimbal.integral_error[0] = 0;
		else
			standard_gimbal.integral_error[0] += standard_gimbal.error[0] * T_gimbal;
		
		standard_gimbal.integral_error[1] += standard_gimbal.error[1] * T_gimbal;
		
		// saturate yaw and pitch integral error
		saturate(&standard_gimbal.integral_error[0], sat_ei_yaw);
		saturate(&standard_gimbal.integral_error[1], sat_ei_pitch);
		
		// compute control signals coming from LQR (without integral error part)
		for (int i = 0; i < 2; i++) {
			cmd_yaw += gimbal_yaw_LQR_K[i] * standard_gimbal.error[2*i];
			cmd_pitch += gimbal_pitch_LQR_K[i] * standard_gimbal.error[2*i+1];
		}
		
		// yaw and pitch angles integral gain
		standard_gimbal_I_gain[0] = (fabs(standard_gimbal.error[0]) <= range_where_trigger_yaw_integral) ? gain_integral_yaw_error : 0;
		standard_gimbal_I_gain[1] = (fabs(standard_gimbal.error[1]) <= range_where_trigger_pitch_integral) ? gain_integral_pitch_error : 0;

		// add integral error part
		cmd_yaw += standard_gimbal_I_gain[0] * standard_gimbal.integral_error[0];
		cmd_pitch += standard_gimbal_I_gain[1] * standard_gimbal.integral_error[1];
		
		// compute control signals coming from LQR (considering the integral error part too)
		standard_gimbal.control_signals[0] = gain_LQR_control_signal_gimbal_yaw * cmd_yaw;
		standard_gimbal.control_signals[1] = gain_LQR_control_signal_gimbal_pitch * cmd_pitch;
		
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
	
	if (id_robot_part == STANDARD_CHASSIS) {
		
		if (IS_SENTRY) {
			sentry_chassis_P_gain = 1.2;
			sentry_chassis_I_gain = 0.001;
		}
		else
			return;
	}
	else if (id_robot_part == SHOOT_MOTORS) {
		
		shoot_P_gain[0] = 2;
		shoot_P_gain[1] = 2;
		shoot_P_gain[2] = 2;
		shoot_P_gain[3] = 2;
		shoot_I_gain[0] = 1;
		shoot_I_gain[1] = 1;
		shoot_I_gain[2] = 1;
		shoot_I_gain[3] = 1;
	}
	else
		return;
}


void PI_controller(int id_robot_part) {
	
	if (id_robot_part == STANDARD_CHASSIS) {
		
		if (IS_SENTRY) {
			
			// update error
			sentry_chassis.error = sentry_chassis.ref_wheel_angular_speed - sentry_chassis.wheel_angular_speed;
			
			// update integral error
			if (sentry_chassis.ref_wheel_angular_speed * sentry_chassis.ref_wheel_angular_speed_prev <= 0)
				sentry_chassis.integral_error = 0;
			else
				sentry_chassis.integral_error += sentry_chassis.error * T_sentry_chassis;
			
			// compute control signals
			sentry_chassis.control_signal = (sentry_chassis_P_gain * sentry_chassis.error) +
																			(sentry_chassis_I_gain * sentry_chassis.integral_error);
		}
		else
			return;
	}
	else if (id_robot_part == SHOOT_MOTORS) {
		
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


float compute_gain_gimbal_mantain_position(int motor, float x1, float distance_x1_x4, float ymax, float epsilon) {
	
	if (!(motor == YAW || motor == PITCH))
		return 1;
	
	int i = (motor == YAW) ? 0 : 1;
	
	if (x1 == standard_gimbal.ref[i])
		return 1;
	
	x4 = (x1 < standard_gimbal.ref[i]) ? (x1 - distance_x1_x4) : (x1 + distance_x1_x4);
	
	x2 = x1 + (x4-x1)/4;
	x3 = x1 + 3*(x4-x1)/4;
	xm = (x1+x4)/2;
	offset_zero = (ymax+1)/2;
	eta = (((offset_zero-1)/(2*epsilon))-1)/(fabs(x2-xm));
	k_coeff = ((1+epsilon-offset_zero)*(1+eta*fabs(x2-xm)))/(eta*(x2-xm));
	
	return min(max(k_coeff*(eta*(standard_gimbal.state_estim[i]-xm))/(1 + eta*fabs(standard_gimbal.state_estim[i]-xm)) + offset_zero, 1), ymax);
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
	float dt 				= pid->dt;
	float ud_prev 	= pid->ud_prev;
	
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
	
	
	// TODO delete comments
//		lpf_update(pid->lpf, ud);
//		ud = lpf_apply(pid->lpf);
		
		//ud = (N*dt)*ud + (1 - N*dt)*ud_prev;
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









