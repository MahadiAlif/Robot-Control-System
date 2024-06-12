#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "arm_math.h"  // Include the CMSIS-DSP library

  /**************************************************************/
 /*  Signatures of EKF equations (to update state and output)  */
/**************************************************************/

/* EKF state equation: x(k+1) = f(x(k), u(k))

	Arguments (in order): x(k), u(k), x(k+1)
*/
typedef void (*ekf_state_eq)(float *, float *, float *);
/* Derivative of EKF state equation

	Arguments (in order): x(k), u(k), x(k+1)
*/
typedef void (*ekf_deriv_state_eq)(float *, float *, float *);
/* EKF output equation: y(k) = h(x(k))

	Arguments (in order): x(k), y(k)
*/
typedef void (*ekf_output_eq)(float *, float *);
/* Derivative of EKF output equation

	Arguments (in order): x(k), y(k)
*/
typedef void (*ekf_deriv_output_eq)(float *, float *);

  /**********************************/
 /*  Extended Kalman Filter (EKF)  */
/**********************************/
typedef struct {
	// Number of states, inputs and outputs
	uint16_t n;  // Number of states
	uint16_t m;  // Number of inputs
	uint16_t q;  // Number of outputs
	// Flags
	uint8_t use_linear_model;  // Flag determining whether to use the linear or (non)linear model.
	uint8_t use_euler_method_for_state_update;  // Flag determining whether to use the Forward Euler method or the Jacobian definition
	// Matrices
	arm_matrix_instance_f32 P;  // State covariance matrix
	arm_matrix_instance_f32 P_pred;  // Prediction of state covariance matrix
	arm_matrix_instance_f32 Q;  // Process noise covariance matrix
    arm_matrix_instance_f32 R;  // Measurement noise covariance matrix
	arm_matrix_instance_f32 S;  // Innovation covariance
    arm_matrix_instance_f32 K;  // Kalman gain
	arm_matrix_instance_f32 F;  // Jacobian of state update function w.r.t. state
	arm_matrix_instance_f32 G;  // Jacobian of state update function w.r.t. input
	arm_matrix_instance_f32 H;  // Jacobian of measurement function w.r.t. state
	arm_matrix_instance_f32 I;  // n-by-n identity matrix
	arm_matrix_instance_f32 F_trans;
	arm_matrix_instance_f32 H_trans;
	arm_matrix_instance_f32 S_inv;
	arm_matrix_instance_f32 H_P_pred;
	arm_matrix_instance_f32 P_pred_H_trans;
	// Vectors
	arm_matrix_instance_f32 x_hat;  // State estimation vector
	arm_matrix_instance_f32 x_pred;  // State prediction vector
	arm_matrix_instance_f32 dy;  // Residual
	arm_matrix_instance_f32 F_x_hat;
	// State and outputs equations, and their derivatives
	ekf_state_eq f;  // User-provided state update function
	ekf_output_eq h;  // User-provided measurement function
	ekf_deriv_state_eq df_dx;  // User-provided derivative of state update function w.r.t. state
	ekf_deriv_state_eq df_du;  // User-provided derivative of state update function w.r.t. input
	ekf_deriv_output_eq dh_dx;  // User-provided derivative of measurement function w.r.t. state
} ekf;

// TODO rewrite doc
/* Initialize the EKF

	Args:
	- filter (ekf *): Pointer to the Extended Kalman Filter structure.
	- n (uint16_t): Number of states.
	- m (uint16_t): Number of inputs.
	- q (uint16_t): Number of outputs.
	- x_hat_init (const float *): Array of initial states.
	- P_init (const float *): Array of initial state covariance matrix.
	- Q (const float *): Process noise covariance matrix.
	- R (const float *): Measurement noise covariance matrix.
	- F (const float *): Jacobian of state update function w.r.t. state
	- G (const float *): Jacobian of state update function w.r.t. input
	- H (const float *): Jacobian of measurement function w.r.t. state
	- f (ekf_state_eq): Discrete-time EKF state equation.
	- h (ekf_state_eq): Discrete-time EKF output equation.
	- df_dx (ekf_deriv_state_eq): Derivative of discrete-time EKF state equation w.r.t. state.
	- df_du (ekf_deriv_state_eq): Derivative of discrete-time EKF state equation w.r.t. input.
	- dh_dx (ekf_deriv_output_eq): Derivative of discrete-time EKF output equation w.r.t. state.
	- use_linear_model (uint8_t): If TRUE (1), the Kalman Filter uses the model given by the provided
		F, G, H matrices. If FALSE (0), the model is given by the (non)linear state and output equations
		linearized at the current plant condition.
	- use_euler_method_for_state_update (uint8_t): If TRUE (1), the state update step is
		performed using Forward Euler method, which is:

			x_pred(k) = f(x_hat(k-1), u(k-1)))
			... where...
			f(x_hat(k-1), u(k-1))) = x_hat(k-1) + dt * fc(x_hat(k-1), u(k-1)))  <-- fc is contiguous-time; f is discrete-time
			
		If FALSE (0), the state update step uses the definition of Jacobian matrix, which is:
			
			x_pred(k) = F(k-1) * x_hat(k-1) + G(k-1) * u(k-1)
			... where...
			F(k-1) = df_dx(x_hat(k-1), u(k-1))
			G(k-1) = df_du(x_hat(k-1), u(k-1))
			
		Therefore:
		- If TRUE, you can avoid to provide arguments: df_du
		- If FALSE, you can avoid to provide arguments: f, h
		
	NOTE: Matrices are implemented by rows as arrays.
	NOTE: If use_euler_method_for_state_update == FALSE, the function pointers f and h can be NULL.
*/
//void ekf_init(ekf *filter, uint16_t n, uint16_t m, uint16_t q, const float *x_hat_init, const float *P_init,
//			  const float *Q, const float *R, const float *F, const float *G, const float *H, ekf_state_eq f,
//		      ekf_output_eq h, ekf_deriv_state_eq df_dx, ekf_deriv_state_eq df_du, ekf_deriv_output_eq dh_dx,
//			  uint8_t use_linear_model, uint8_t use_euler_method_for_state_update);
void ekf_init(ekf *filter, uint16_t n, uint16_t m, uint16_t q, uint8_t use_linear_model, uint8_t use_euler_method_for_state_update,
			  const float *x_hat, const float *P,  const float *Q, const float *R, const float *F, const float *G, const float *H,
			  const float *P_pred, const float *S, const float *K, const float *I, const float *x_pred, const float *dy, const float *F_trans,
			  const float *H_trans, const float *S_inv, const float *F_x_hat, const float *H_P_pred, const float *P_pred_H_trans,
			  ekf_state_eq f, ekf_output_eq h, ekf_deriv_state_eq df_dx, ekf_deriv_state_eq df_du, ekf_deriv_output_eq dh_dx);

// TODO rewrite doc
/* Update the EKF

	Args:
	- filter (ekf *): Pointer to the Extended Kalman Filter structure.
	- u (float *): Array of inputs at time k-1 (where k is the current time instant).
	- y (float *): Array of outputs at time k (where k is the current time instant).

	NOTE: u is u(k-1); y is y(k); The current time instant is k.
	NOTE: Matrices are implemented by rows as arrays.
*/
void ekf_update(ekf *filter, const float *plant_u_prev, const float *plant_y_curr);

#endif