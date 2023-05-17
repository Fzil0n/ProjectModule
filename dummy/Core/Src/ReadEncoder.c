/*
 * ReadEncoder.c
 *
 *  Created on: May 17, 2023
 *      Author: tanawatp
 */
#include "ReadEncoder.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

QEIStructureTypedef QEIData = {0};

uint64_t _micros = 0;

uint16_t PPR = 8192;

uint32_t samplingTime = 10000; //us  10 Hz

//PWM
uint8_t MotorSetDuty = 0;
uint8_t Pulse_Compare = 0;
uint8_t DIR = 0;

extern  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ //get time period
	if(htim == & htim5){
		_micros += UINT32_MAX;
	}
}
uint64_t micros(){ //get time in micros
	return __HAL_TIM_GET_COUNTER(&htim5)+ _micros;
}

void QEIEncoderPositionVelocity_Update(){
	//collect data
	QEIData.timestamp[0] = micros();
	uint32_t couterPosition = __HAL_TIM_GET_COUNTER(&htim3);
	QEIData.data[0] = couterPosition;

	//calculation
	QEIData.QEIPosition = couterPosition % PPR;

	int32_t diffPosition = QEIData.data[0] - QEIData.data[1];
	float difftime = (QEIData.timestamp[0] - QEIData.timestamp[1]);

	//unwarp
	if(diffPosition > UINT16_MAX>>1) diffPosition -= UINT16_MAX;
	if(diffPosition < -(UINT16_MAX>>1)) diffPosition += UINT16_MAX;

	//calculate
	QEIData.QEIVelocity = (diffPosition * 1000000)/difftime;

	//Delay
	QEIData.data[1] = QEIData.data[0];
	QEIData.timestamp[1] = QEIData.timestamp[0];
}


