/*
 * KalmanFilter.c
 *
 *  Created on: May 17, 2023
 *      Author: tanawatp
 */
#include "arm_math.h"


#define STATE_DIM 3
#define MEASUREMENT_DIM 1

void kalman_filter(float32_t* x_hat, float32_t* P, float32_t var_Q, float32_t R, float32_t z)
{
    // System matrices
	float32_t dt = 0.01;
    float32_t A[STATE_DIM * STATE_DIM] = {1, dt, 0.5 * dt * dt, 0, 1, dt, 0, 0, 1};
    float32_t B[STATE_DIM] = {0, 0, 0};
    float32_t C[MEASUREMENT_DIM * STATE_DIM] = {0, 1, 0};
    float32_t D = 0;
    float32_t Q[9]; //G*GT*var_Q
    float32_t G[STATE_DIM] = {dt * dt * dt / 6, 0.5 * dt * dt, dt};
    float32_t GT[STATE_DIM];
    float32_t I[9] = {
    		1, 0, 0,
    		0, 1, 0,
			0, 0, 1
    };//Identity

    // Intermediate matrices and vectors
    float32_t x_hat_minus[STATE_DIM];
    float32_t P_minus[STATE_DIM * STATE_DIM];
    float32_t S[MEASUREMENT_DIM * MEASUREMENT_DIM];
    float32_t K[STATE_DIM * MEASUREMENT_DIM];
    float32_t temp3x3A[STATE_DIM * STATE_DIM];//9
    float32_t temp3x3B[STATE_DIM * STATE_DIM];//9
    float32_t temp3x1[STATE_DIM * MEASUREMENT_DIM];//3
    float32_t temp1x3[STATE_DIM * MEASUREMENT_DIM];//3
    float32_t temp1x1;//1

    // Predict state estimate: x_hat_minus = A * x_hat + B * u
    arm_matrix_instance_f32 mat_A, mat_x_hat, mat_x_hat_minus, mat_B, mat_u, mat_GT, mat_G,eye;
    arm_matrix_instance_f32 mat_P, mat_P_minus, mat_Q;
    arm_matrix_instance_f32 mat_C, mat_R, mat_S, mat_K, mat_temp3x3A,mat_temp3x3B, mat_temp3x1,mat_temp1x3, mat_temp1x1;

    arm_mat_init_f32(&mat_A, STATE_DIM, STATE_DIM, (float32_t*) &A);//3x3
    arm_mat_init_f32(&mat_x_hat, STATE_DIM, 1, x_hat);
    arm_mat_init_f32(&mat_x_hat_minus, STATE_DIM, 1, x_hat_minus);
    arm_mat_init_f32(&mat_B, STATE_DIM, 1, B);
    //arm_mat_init_f32(&mat_u, 1, 1, NULL);  // Set the input control vector if needed
    arm_mat_init_f32(&mat_P, STATE_DIM, STATE_DIM, P);//3x3
    arm_mat_init_f32(&mat_P_minus, STATE_DIM, STATE_DIM, P_minus);//3x3
    arm_mat_init_f32(&mat_Q, STATE_DIM, STATE_DIM,(float32_t*) &Q);//3x3
    arm_mat_init_f32(&mat_C, MEASUREMENT_DIM, STATE_DIM, (float32_t*) &C);//1x3
    arm_mat_init_f32(&mat_R, MEASUREMENT_DIM, MEASUREMENT_DIM, &R);//1x1
    arm_mat_init_f32(&mat_S, MEASUREMENT_DIM, MEASUREMENT_DIM, (float32_t*) &S);//1x1
    arm_mat_init_f32(&mat_K, STATE_DIM, MEASUREMENT_DIM, (float32_t*) &K);//3x1
    arm_mat_init_f32(&mat_temp3x3A, STATE_DIM, STATE_DIM, (float32_t*) &temp3x3A);//3x3
    arm_mat_init_f32(&mat_temp3x3B, STATE_DIM, STATE_DIM, (float32_t*) &temp3x3B);//3x3
    arm_mat_init_f32(&mat_temp3x1, STATE_DIM, MEASUREMENT_DIM, (float32_t*) &temp3x1);//3x1
    arm_mat_init_f32(&mat_temp1x3, MEASUREMENT_DIM, STATE_DIM, (float32_t*) &temp3x1);//1x3
    arm_mat_init_f32(&mat_temp1x1, MEASUREMENT_DIM, MEASUREMENT_DIM, (float32_t*) &temp1x1);//1x1
    arm_mat_init_f32(&mat_G, STATE_DIM, 1, (float32_t*) &G);//3x1
    arm_mat_init_f32(&mat_GT, 1, STATE_DIM, (float32_t*) &GT);//1x3
    arm_mat_init_f32(&eye, 3, 3, (float32_t*) &I);//1x3

    //Process model: x_hat_minus = mat_A*x_hat
    arm_mat_mult_f32(&mat_A, &mat_x_hat, &mat_x_hat_minus);			//A*X
    // If an input control vector is used:
    // arm_mat_mult_f32(&mat_B, &mat_u, &mat_temp3x3A);
    // arm_mat_add_f32(&mat_x_hat_minus, &mat_temp3x3A, &mat_x_hat_minus);

    // Predict covariance matrix: P_minus = A * P * A^T + Q
    arm_mat_trans_f32(&mat_A, &mat_temp3x3A);					    //AT
    arm_mat_mult_f32(&mat_A, &mat_P, &mat_temp3x3B);			    //A*P
    arm_mat_mult_f32(&mat_temp3x3B, &mat_temp3x3A, &mat_P_minus);	//A*P*AT
    arm_mat_trans_f32(&mat_G, &mat_GT);								//GT
    arm_mat_mult_f32(&mat_G, &mat_GT, &mat_Q);						//G*GT
    arm_mat_scale_f32(&mat_Q, var_Q, &mat_Q);						//G*GT*var_q
    arm_mat_add_f32(&mat_P_minus, &mat_Q, &mat_P_minus);			//A * P * A^T + Q

    // Calculate innovation covariance: S = C * P_minus * C^T + R
    arm_mat_mult_f32(&mat_C, &mat_P_minus, &mat_temp1x3);			//C*P
    arm_mat_trans_f32(&mat_C, &mat_temp3x1);						//CT
    arm_mat_mult_f32(&mat_temp1x3, &mat_temp3x1, &mat_temp1x1);		//C*P*C^T
    arm_mat_add_f32(&mat_temp1x1, &mat_R, &mat_S);					//C*P*C^T + R

    // Calculate Kalman gain: K = P_minus * C^T * S^(-1)
    arm_mat_inverse_f32(&mat_S, &mat_temp1x1);						//inv(S)
    arm_mat_mult_f32(&mat_P_minus, &mat_temp3x1, &mat_temp3x3A);	//P*CT
    arm_mat_mult_f32(&mat_temp3x3A, &mat_temp1x1, &mat_K);			//P*CT*inv(S)

    // Update state estimate: x_hat = x_hat_minus + K * (z - C * x_hat_minus)
    arm_mat_mult_f32(&mat_C, &mat_x_hat_minus, &mat_temp1x1);		//C*X
    //float32_t innovation = z - temp1x1;								//Z-C*X
    arm_mat_scale_f32(&mat_K, z - temp1x1, &mat_temp3x1);			//K*(Z-C*X)
    arm_mat_add_f32(&mat_x_hat_minus, &mat_temp3x1, &mat_x_hat);	//X - K*(Z-C*X) ========> X estimate

    // Update covariance matrix: P = (I - K * C) * P_minus
    arm_mat_mult_f32(&mat_K, &mat_C, &mat_temp3x3B);				//K*C
    arm_mat_sub_f32(&eye, &mat_P_minus, &mat_temp3x3A);				//I - K*C
    arm_mat_mult_f32(&mat_temp3x3A, &mat_P_minus, &mat_P);			//(I - K*C)*P ===========> P estimate
}

