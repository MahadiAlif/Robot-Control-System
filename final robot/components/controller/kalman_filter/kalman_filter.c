#include "kalman_filter.h"
#include <stdlib.h>


//void ekf_init(ekf *filter, uint16_t n, uint16_t m, uint16_t q, const float *x_hat_init, const float *P_init,
//			  const float *Q, const float *R, const float *F, const float *G, const float *H, ekf_state_eq f,
//		      ekf_output_eq h, ekf_deriv_state_eq df_dx, ekf_deriv_state_eq df_du, ekf_deriv_output_eq dh_dx,
//			  uint8_t use_linear_model, uint8_t use_euler_method_for_state_update)
void ekf_init(ekf *filter, uint16_t n, uint16_t m, uint16_t q, uint8_t use_linear_model, uint8_t use_euler_method_for_state_update,
			  const float *x_hat, const float *P,  const float *Q, const float *R, const float *F, const float *G, const float *H,
			  const float *P_pred, const float *S, const float *K, const float *I, const float *x_pred, const float *dy, const float *F_trans,
			  const float *H_trans, const float *S_inv, const float *F_x_hat, const float *H_P_pred, const float *P_pred_H_trans,
			  ekf_state_eq f, ekf_output_eq h, ekf_deriv_state_eq df_dx, ekf_deriv_state_eq df_du, ekf_deriv_output_eq dh_dx)
{
	// Initialize number of states, inputs and outputs
	filter->n = n;
	filter->m = m;
	filter->q = q;
	// Initialize flags
	filter->use_linear_model = use_linear_model;
	filter->use_euler_method_for_state_update = use_euler_method_for_state_update;
	// Allocate matrices memory
	arm_mat_init_f32(&filter->P, n, n, P);//(float *) malloc(n * n * sizeof(float)));
	arm_mat_init_f32(&filter->P_pred, n, n, P_pred);//(float *) malloc(n * n * sizeof(float)));
	arm_mat_init_f32(&filter->Q, n, n, Q);//(float *) malloc(n * n * sizeof(float)));
    arm_mat_init_f32(&filter->R, q, q, R);//(float *) malloc(q * q * sizeof(float)));
	arm_mat_init_f32(&filter->S, q, q, S);//(float *) malloc(q * q * sizeof(float)));
    arm_mat_init_f32(&filter->K, n, q, K);//(float *) malloc(n * q * sizeof(float)));
	arm_mat_init_f32(&filter->F, n, n, F);//(float *) malloc(n * n * sizeof(float)));
	arm_mat_init_f32(&filter->G, n, m, G);//(float *) malloc(n * m * sizeof(float)));
	arm_mat_init_f32(&filter->H, q, n, H);//(float *) malloc(q * n * sizeof(float)));
	arm_mat_init_f32(&filter->I, n, n, I);//(float *) malloc(n * n * sizeof(float)));
	arm_mat_init_f32(&filter->F_trans, n, n, F_trans);
	arm_mat_init_f32(&filter->H_trans, q, n, H_trans);
	arm_mat_init_f32(&filter->S_inv, q, q, S_inv);
	arm_mat_init_f32(&filter->H_P_pred, q, n, H_P_pred);
	arm_mat_init_f32(&filter->P_pred_H_trans, n, q, P_pred_H_trans);
	// Allocate vectors memory
	arm_mat_init_f32(&filter->x_hat,	n, 1, x_hat);//(float *) malloc(n * sizeof(float)));
	arm_mat_init_f32(&filter->x_pred, 	n, 1, x_pred);//(float *) malloc(n * sizeof(float)));
    arm_mat_init_f32(&filter->dy, 		q, 1, dy);//(float *) malloc(q * sizeof(float)));
	arm_mat_init_f32(&filter->F_x_hat, 	n, 1, F_x_hat);
	// Initialize matrices
//	memcpy((void *) filter->P.pData, (const void *) P_init, (size_t) n * n * sizeof(float));
//	memcpy((void *) filter->Q.pData, (const void *) Q, (size_t) n * n * sizeof(float));
//	memcpy((void *) filter->R.pData, (const void *) R, (size_t) q * q * sizeof(float));
//	memcpy((void *) filter->F.pData, (const void *) F, (size_t) n * n * sizeof(float));
//	memcpy((void *) filter->G.pData, (const void *) G, (size_t) n * m * sizeof(float));
//	memcpy((void *) filter->H.pData, (const void *) H, (size_t) q * n * sizeof(float));
	for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            // Set diagonal elements of Identity matrix to 1, others to 0
            filter->I.pData[i * n + j] = (i == j) ? 1 : 0;
        }
    }
	// Initialize vectors
//	memcpy((void *) filter->x_hat.pData, (const void *) x_hat_init, (size_t) n);
	// Initialize state and outputs equations, and their derivatives
	filter->f = f;
    filter->h = h;
	filter->df_dx = df_dx;
	filter->df_du = df_du;
    filter->dh_dx = dh_dx;
}

void ekf_update(ekf *filter, const float *plant_u_prev, const float *plant_y_curr)
{
	// Define useful matrices and vectors
//	arm_matrix_instance_f32 uk_1;
//	arm_matrix_instance_f32 yk;
//	arm_matrix_instance_f32 F_trans;
//	arm_matrix_instance_f32 H_trans;
//	arm_matrix_instance_f32 S_inv;
//	arm_matrix_instance_f32 tmp_n1;
//	arm_matrix_instance_f32 tmp_qn;
//	arm_matrix_instance_f32 tmp_nq;
	// Allocate memory for useful matrices
//	arm_mat_init_f32(&uk_1, filter->m, 1, (float *)u);
//	arm_mat_init_f32(&yk, filter->q, 1, (float *)y);
//	arm_mat_init_f32(&F_trans, filter->n, filter->n, (float *)NULL);
//	arm_mat_init_f32(&H_trans, filter->n, filter->q, (float *)NULL);
//	arm_mat_init_f32(&S_inv, filter->q, filter->q, (float *)NULL);
//	arm_mat_init_f32(&tmp_n1, filter->n, 1, (float *)NULL);
//	arm_mat_init_f32(&tmp_qn, filter->q, filter->n, (float *)NULL);
//	arm_mat_init_f32(&tmp_nq, filter->n, filter->q, (float *)NULL);
	
	arm_matrix_instance_f32 u;
	arm_matrix_instance_f32 y;
	arm_mat_init_f32(&u, filter->m, 1, (float *) plant_u_prev);
	arm_mat_init_f32(&y, filter->q, 1, (float *) plant_y_curr);
	
    /* Prediction Step */
	
	if (!filter->use_linear_model) {
		// F(k-1) = df_dx(x_hat(k-1), u(k-1))
		filter->df_dx(filter->x_hat.pData, u.pData, filter->F.pData);
	}
	
    // Compute x_pred(k)
	if (filter->use_euler_method_for_state_update) {
		/*
		x_pred(k) = f(x_hat(k-1), u(k-1)))
		... where...
		f(x_hat(k-1), u(k-1))) = x_hat(k-1) + dt * fc(x_hat(k-1), u(k-1)))  <-- fc is contiguous-time; f is discrete-time
		*/
		filter->f(filter->x_hat.pData, u.pData, filter->x_pred.pData);
	}
	else {
		if (!filter->use_linear_model) {
			// G(k-1) = df_dx(x_hat(k-1), u(k-1))
			filter->df_du(filter->x_hat.pData, u.pData, filter->G.pData);
		}
		/*
		x_pred(k) = F(k-1) * x_hat(k-1) + G(k-1) * u(k-1)
		... where...
		F(k-1) = df_dx(x_hat(k-1), u(k-1))
		G(k-1) = df_du(x_hat(k-1), u(k-1))
		*/
		arm_mat_mult_f32(&filter->F, &filter->x_hat, &filter->F_x_hat);
		arm_mat_mult_f32(&filter->G, &u, &filter->x_pred);
		arm_mat_add_f32(&filter->F_x_hat, &filter->x_pred, &filter->x_pred);
	}

    // P_pred(k) = F(k-1) * P(k-1) * F(k-1)' + Q
	arm_mat_trans_f32(&filter->F, &filter->F_trans);
	arm_mat_mult_f32(&filter->F, &filter->P, &filter->P_pred);
	arm_mat_mult_f32(&filter->P_pred, &filter->F_trans, &filter->P_pred);
	arm_mat_add_f32(&filter->P_pred, &filter->Q, &filter->P_pred);

    /* Update Step */
	
	if (!filter->use_linear_model) {
		// H(k) = dh_dx(x_pred(k))
		filter->dh_dx(filter->x_pred.pData, filter->H.pData);
	}

    // S(k) = H(k) * P_pred(k) * H(k)' + R
	arm_mat_trans_f32(&filter->H, &filter->H_trans);
    arm_mat_mult_f32(&filter->H, &filter->P_pred, &filter->H_P_pred);
    arm_mat_mult_f32(&filter->H_P_pred, &filter->H_trans, &filter->S);
    arm_mat_add_f32(&filter->S, &filter->R, &filter->S);

    // K(k) = P_pred(k) * H(k)' * inv(S(k))
    arm_mat_inverse_f32(&filter->S, &filter->S_inv);
    arm_mat_mult_f32(&filter->P_pred, &filter->H_trans, &filter->P_pred_H_trans);
    arm_mat_mult_f32(&filter->P_pred_H_trans, &filter->S_inv, &filter->K);
	
	// dy(k) = y(k) - h(x_pred(k))
	if (filter->use_linear_model) {
		arm_mat_mult_f32(&filter->H, &filter->x_pred, &filter->dy);
	}
	else {
		filter->h(filter->x_pred.pData, filter->dy.pData);
	}
    arm_mat_sub_f32(&y, &filter->dy, &filter->dy);

    // x_hat(k) = x_pred(k) + K(k) * dy(k)
    arm_mat_mult_f32(&filter->K, &filter->dy, &filter->x_hat);
    arm_mat_add_f32(&filter->x_pred, &filter->x_hat, &filter->x_hat);

    // P(k) = (I - K(k) * H(k)) * P_pred(k)
    arm_mat_mult_f32(&filter->K, &filter->H, &filter->P);
    arm_mat_sub_f32(&filter->I, &filter->P, &filter->P);
    arm_mat_mult_f32(&filter->P, &filter->P_pred, &filter->P);
}


