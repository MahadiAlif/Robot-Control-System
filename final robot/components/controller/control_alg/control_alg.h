#ifndef CONTROL_ALG_H
#define CONTROL_ALG_H

#include <stdint.h>
#include "control_util.h"
#include "robot_config.h"

/************************************************************************************************/
/*																						MACROS																						*/
/************************************************************************************************/


/****************************************************************************/

// robot's part ID
#define BR_CHASSIS 1
#define STANDARD_GIMBAL 2
#define STANDARD_CHASSIS 3
#define SHOOT_MOTORS 4
#define REV_MOTOR 5


// gimbal's motors
#define YAW 1
#define PITCH 2

// gimbal
#define MAX_GIMBAL_YAW_CONTROL_SIGNAL_AMPLITUDE 15000			// defined in range [0,+30000]
#define MAX_GIMBAL_PITCH_CONTROL_SIGNAL_AMPLITUDE 25000		// defined in range [0,+30000]
#define MAX_GIMBAL_YAW_MANTAIN_POSITION_CONTROL_SIGNAL_AMPLITUDE 15000			// defined in range [0,+30000]
#define MAX_GIMBAL_PITCH_MANTAIN_POSITION_CONTROL_SIGNAL_AMPLITUDE 20000		// defined in range [0,+30000]
#define MAX_GIMBAL_YAW_INTEGRATIVE_CONTROL_PART_AMPLITUDE 0.16f			// saturation value of gimbal's yaw integral control part
#define MAX_GIMBAL_PITCH_INTEGRATIVE_CONTROL_PART_AMPLITUDE 0.1f		// saturation value of gimbal's pitch integral control part
#define NEED_TO_TEST_SENTRY_FUNCTIONALITIES_ON_BALANCING 0		// used if we don't have sentry available yet and we wanna test the AI detect on the BR

// standard chassis
#define STD_CHASSIS_U_SAT 10000				// saturation value for the control signal of standard robot chassis

// shoot and rev
#define MAX_SHOOT_CONTROL_SIGNAL_AMPLITUDE 10000		// defined in range [0,+10000]
#define MAX_REV_CONTROL_SIGNAL_AMPLITUDE 10000
#define MAX_REV_INTEGRATIVE_CONTROL_PART_AMPLITUDE 0.06f	// this value was chosen by looking at the integral error that is reached in 1s

// BR chassis
#define MAX_BR_CHASSIS_OVERALL_U 13000
#if IS_STD
#define CENTER_ENCODER_ANGLE_PITCH	UNDEF*RAD_ENCODER_ANGLE_RATIO
#elif IS_BR2
#define CENTER_ENCODER_ANGLE_PITCH	8166/*13*/*RAD_ENCODER_ANGLE_RATIO
#elif IS_SENTRY
#define CENTER_ENCODER_ANGLE_PITCH	UNDEF*RAD_ENCODER_ANGLE_RATIO
#endif

// control/filtering constants
#define filter_time_constant_gimbal 0.003
#define torque_constant_GM6020_motor 0.741	// measurement unit = [N*m/A]
#define M3508_gearbox_ratio 0.052075		// 0.052075 = (187/3591) = (from_rad_motor_to_rad_wheel)
#define M2006_gearbox_ratio 0.027778		// 0.027778 = (1/36)
#define rpm_to_radsec 0.10472					  // 0.10472 = 2*pi/60
#define RAD_ENCODER_ANGLE_RATIO 0.000767		// ratio between angle in radiants and encoder value in range [0,8191] ==> 0.000767 = 2*pi/8192
#if IS_STD
#define CHASSIS_YAW_ENCODER_ZERO_OFFSET 	/*7752*/3977*RAD_ENCODER_ANGLE_RATIO
#elif IS_BR2
#define CHASSIS_YAW_ENCODER_ZERO_OFFSET 	4394*RAD_ENCODER_ANGLE_RATIO
#elif IS_SENTRY
#define CHASSIS_YAW_ENCODER_ZERO_OFFSET 	/*1593*/7255*RAD_ENCODER_ANGLE_RATIO
#endif


// keyboard/mouse values
#define MAX_MOUSE_SPEED_X 1	// measurement unit = [m/s]
#define MAX_MOUSE_SPEED_Y 1	// measurement unit = [m/s]


/* system identification */
#define SYSID_BR_CHASSIS 				0		// enable data sampling for system identification on balancing robot chassis
#define SYSID_STD_CHASSIS 				0		// enable data sampling for system identification on standard robot chassis
#define SYSID_SENTRY_CHASSIS 			0		// enable data sampling for system identification on sentry robot chassis
#define SYSID_GIMBAL 					0		// enable data sampling for system identification on gimbal
#define SYSID_WN						0		// 1 if use white noise for system identification, 0 otherwise
#define SYSID_RAND_BIN					0		// 1 if use random binary distributed signal for system identification, 0 otherwise
#define SYSID_RAND_UNIF					0		// 1 if use random uniformly distributed signal for system identification, 0 otherwise
#define SYSID_RC						0		// 1 if use remote controller commands for system identification, 0 otherwise
#define SYSID_SEND_SAMPLES_PERIOD 		1e-1	// time period for sending inputs to motors
#define SYSID_ACQUIRE_SAMPLES_PERIOD 	8e-3	// data sampling period
#define SYSID_SENT_NUM_SAMPLES 			200		// number of data to be sent to motors
#define SYSID_ACQUIRED_NUM_SAMPLES 		1100	// number of data to be sampled



/**************************************************************************************************/
/*																						VARIABLES																						*/
/**************************************************************************************************/

/************************************************************************************************************************
	NAME: robot
	
	DESCRIPTION: data structure that represents a robot (including its state variables, inputs to its motors, references, error w.r.t. references)

								   ____________	           _________
			+		  error	      |			   |    u	  |			|
	ref ------>O----------------->| CONTROLLER |--------->|  ROBOT  |----------> x
			   |-				  |____________|		  |_________|	  |
               |														  |
			   |__________________________________________________________|
	
	
	---------- Standard robot chassis ----------
	states x:
		0: angular velocity of front-left wheel [rad/s]
		1: angular velocity of front-right wheel [rad/s]
		2: angular velocity of back-right wheel [rad/s]
		3: angular velocity of back-left wheel [rad/s]
		4: yaw relative angle between chassis and gimbal orientation [rad]
	inputs u:
		0: command to front-left wheel (in range [-16384, +16384])
		1: command to front-right wheel (in range [-16384, +16384])
		2: command to back-right wheel (in range [-16384, +16384])
		3: command to back-left wheel (in range [-16384, +16384])
	
	---------- Balancing robot chassis ----------
	states x:
		0: linear position of left wheel [m]
		1: linear velocity of left wheel [m/s]
		2: linear position of right wheel [m]
		3: linear velocity of right wheel [m/s]
		4: angular position of chassis [rad]
		5: angular velocity of chassis [rad/s]
		6: yaw relative angle between chassis and gimbal orientation [rad]
		7: linear position of chassis COM (Center of Mass) [m]
		8: linear velocity of chassis COM (Center of Mass) [m/s]
	inputs u:
		0: command to left wheel (in range [-16384, +16384])
		1: command to right wheel (in range [-16384, +16384])
	
	---------- Gimbal ----------
	states x:
		0: yaw position [rad]
		1: pitch position [rad]
		2: yaw velocity [rad/s]
		3: pitch velocity [rad/s]
	inputs u:
		0: command to yaw motor (in range [-30000, +30000])
		1: command to pitch motor (in range [-30000, +30000])
	
	
	NOTE:
	- for M3508 motor + C620 speed controller:	command in [-16384, +16384] ==> current in range [-20A, +20A]
	- for GM6020 motor:													command in [-30000, +30000] ==> voltage in range [-24V, +24V] approximately

************************************************************************************************************************/
struct robot {
	
	uint8_t n;															// number of robot states
	uint8_t m;															// number of robot inputs
	
	float x[MAX_ROBOT_STATES];							// current robot states
	float x_prev[MAX_ROBOT_STATES];					// previous robot states
	float u[MAX_ROBOT_INPUTS];							// current robot inputs
	float u_prev[MAX_ROBOT_INPUTS];					// previous robot inputs
	
	float ref[MAX_ROBOT_STATES];						// current reference for robot states
	float ref_prev[MAX_ROBOT_STATES];				// previous reference for robot states
	
	float error[MAX_ROBOT_STATES];					// current error w.r.t. reference
	float error_prev[MAX_ROBOT_STATES];			// previous error w.r.t. reference
	float integr_error[MAX_ROBOT_STATES];		// integral error w.r.t. reference
	float deriv_error[MAX_ROBOT_STATES];		// derivative error w.r.t. reference
};


/************************************************************************************************************************
	NAME: PID_
	
	DESCRIPTION: PID controller (Proportional, Integrative, Derivative)

	NOTE:
	- the '_' character is just to avoid definition conflicts with other pre-defined functions
************************************************************************************************************************/
struct PID_ {
	
	float Kp;					// Proportional part
	float Ki;					// Integrative part
	float Kd;					// Derivative part
	float dt;					// controller time period
	float up_prev;		// previous Proportional control signal of PID
	float ui_prev;		// previous Integrative control signal of PID
	float ud_prev;		// previous Derivative control signal of PID
};


/************************************************************************************************************************
	NAME: LQR
	
	DESCRIPTION: LQR controller
************************************************************************************************************************/
struct LQR {
	
	float K[MAX_ROBOT_INPUTS * MAX_ROBOT_STATES];			// LQR matrix gain
	float Ki[MAX_ROBOT_INPUTS * MAX_ROBOT_STATES];		// integral gain (for steady-state convergence to the reference)
};


/**************************************************************************************************/
/*																						FUNCTIONS																						*/
/**************************************************************************************************/

// control algorithms
void LQR_gain_init(int id_robot_part);
void LQR_controller(int id_robot_part);
void PI_gain_init(int id_robot_part);
void PI_controller(int id_robot_part);

float exp_error_gain(float error, float x1, float x2, float x3, float y1, float y2, float y3);


/************************************************************************************************************************
	NAME: PID_init_
	
	DESCRIPTION: initializes the P,I,D,N,dt parameters of the given PID controller

	ARGUMENTS:
	- pid:		pointer to the PID to be inizialized
	- Kp:			P parameter of PID
	- Ki:			I parameter of PID
	- Kd:			D parameter of PID
	- dt:			controller time period
************************************************************************************************************************/
void PID_init_(struct PID_ *pid, float Kp, float Ki, float Kd, float dt);


/************************************************************************************************************************
	NAME: LQR_init
	
	DESCRIPTION: initializes the K matrix gain of the given LQR controller

	ARGUMENTS:
	- lqr:	pointer to the LQR to be inizialized
	- K:		K matrix gain
	- Ki:		Ki integral matrix gain (for steady-state convergence to the reference)
	- m:		number of inputs of the robot to be controller by the LQR
	- n:		number of states of the robot to be controller by the LQR
	
	NOTE: K and Ki are "m x n" matrices (m rows, n columns) embedded in arrays of size m*n
************************************************************************************************************************/
void LQR_init(struct LQR *lqr, float *K, float *Ki, uint8_t m, uint8_t n);


/************************************************************************************************************************
	NAME: PID_control
	
	DESCRIPTION: performs the PID control using the given errors w.r.t. references and the given PID controller

	ARGUMENTS:
	- pid:			pointer to the PID controller structure
	- ep:				proportional error (to be multiplied by the Proportional gain of PID)
	- ei:				integral error (to be multiplied by the Integral gain of PID)
	- ed:				derivative error (to be multiplied by the Derivative gain of PID)
	- sat_up:		saturation value for the Proportional component of final control signal
	- sat_ui:		saturation value for the Integral component of final control signal
	- sat_ud:		saturation value for the Derivative component of final control signal
	- fir_up:		pointer to the FIR low-pass filter of Proportial part
	- fir_ui:		pointer to the FIR low-pass filter of Integrative part
	- fir_ud:		pointer to the FIR low-pass filter of Derivative part
	
	The final control signal is ...
		u = up + ui + ud
	... where ...
		- up = Kp*ep saturated in range [-sat_up, +sat_up]
		- ui = Ki*ei saturated in range [-sat_ui, +sat_ui]
		- ud = Kd*ed saturated in range [-sat_ud, +sat_ud]
		- the Derivative control signal "ud" is filtered by the FIR LPF (if it was provided in the PID initialization)
	
	NOTE:
	- if you don't want to define a saturation value, just write UNDEF
	- the Derivative term will be filtered if the pointer to the FIR low-pass filter of the PID controller is not NULL				// TODO modify comments
************************************************************************************************************************/
float PID_control(struct PID_ *pid, float ep, float ei, float ed, float sat_up, float sat_ui, float sat_ud, struct fir *fir_up, struct fir *fir_ui, struct fir *fir_ud);


/************************************************************************************************************************
	NAME: LQR_control
	
	DESCRIPTION: performs LQR control on the given robot data structure, using the given LQR controller

	ARGUMENTS:
	- robot:	pointer to the robot structure to be controlled
	- lqr:		pointer to the LQR structure
	
	The final control signal is:
	u[0] = K[0][0]*error[0] + ... + K[0][n-1]*error[n-1] + Ki[0][0]*integr_error[0] + ... + Ki[0][n-1]*integr_error[n-1];
	u[1] = K[1][0]*error[0] + ... + K[1][n-1]*error[n-1] + Ki[1][0]*integr_error[0] + ... + Ki[1][n-1]*integr_error[n-1];
	...
	u[m-1] = K[m-1][0]*error[0] + ... + K[m-1][n-1]*error[n-1] + Ki[m-1][0]*integr_error[0] + ... + Ki[m-1][n-1]*integr_error[n-1];
	
	NOTE:
	- error[i] = ref[i] - x[i];
	- integr_error[i] is the discrete integral of error[i]
	- if error[j] should NOT contribute to u[i], just set K[i][j] = 0
	- if integr_error[j] should NOT contribute to u[i], just set Ki[i][j] = 0
************************************************************************************************************************/
void LQR_control(struct robot *robot, struct LQR *lqr);

/************************************************************************************************************************
	NAME: tf_resp
	
	DESCRIPTION: computes the response of a transfer function to a given input

	ARGUMENTS:
	- tf:		pointer to the tf
	- u:		input at current time (i.e. u[k])

	The tf response y[k] is computed by:
	
	y[k] = (b[n]*u[k] + b[n-1]*u[k-1] + b[n-2]*u[k-2] + ... + b[0]*u[k-n] - ...
								... - a[n-1]*y[k-1] - b[n-2]*y[k-2] - ... - a[0]*y[k-n]) / a[n];
	
	
	NOTE: if m < n, then inputs are delayed by n-m time units ==> y[k] does NOT depend on u[k], u[k-1], ..., u[k-n+m+1]
	
					b[m]*z^{m-n} + b[m-1]*z^{m-n-1} + ... + b[0]*z^{-n}
	TF = --------------------------------------------------------
			 a[n] + a[n-1]*z^{-1} + a[n-2]*z^{-2} + ... + a[0]*z^{-n}
************************************************************************************************************************/
float tf_resp(struct tf *tf, float u);


/************************************************************************************************************************
	NAME: ref_error
	
	DESCRIPTION: computes the error, integral error and derivative error of the given robot w.r.t. the references (for all its states)

	ARGUMENTS:
	- robot:		pointer to the robot structure
	- dt:				time period of the control algorithm execution for the given robot (used to compute the discrete integral and derivative error)
************************************************************************************************************************/
void ref_error(struct robot *robot, float dt);


/************************************************************************************************************************
	NAME: ref_error_state
	
	DESCRIPTION: same as ref_error, but computes just the errors of the given state

	ARGUMENTS:
	- robot:		pointer to the robot structure
	- state:		index of the robot's state that you want to compute the errors
	- dt:				time period of the control algorithm execution for the given robot (used to compute the discrete integral and derivative error)
************************************************************************************************************************/
void ref_error_state(struct robot *robot, int state, float dt);



#endif
