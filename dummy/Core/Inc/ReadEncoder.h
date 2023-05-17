/*
 * ReadEncoder.h
 *
 *  Created on: May 17, 2023
 *      Author: tanawatp
 */

#ifndef INC_READENCODER_H_
#define INC_READENCODER_H_

#include "main.h"

typedef struct {
	uint32_t data[2];
	uint64_t timestamp[2];

	float QEIPosition;
	float QEIVelocity;
}QEIStructureTypedef;

uint64_t micros();
void QEIEncoderPositionVelocity_Update();



#endif /* INC_READENCODER_H_ */
