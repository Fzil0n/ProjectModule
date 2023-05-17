/*
 * KalmanFilter.h
 *
 *  Created on: May 17, 2023
 *      Author: tanawatp
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_

#include "arm_math.h"
float test = 5;
float32_t x_hat[3];  // State estimate vector [x, dx, d2x]
float32_t P[9];      // Covariance matrix
float32_t Q[9];      // Process noise covariance matrix
float32_t R = 0;   // Measurement noise variance
float32_t varQ = 0; 	//
float32_t z = 0;   // Measurement value
void kalman_filter(float32_t* x_hat, float32_t* P, float32_t* Q, float32_t R, float32_t z);


#endif /* INC_KALMANFILTER_H_ */
