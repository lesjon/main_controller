/*
 * commsfpga.h
 *
 *  Created on: 1 Nov 2016
 *      Author: rik
 */

#ifndef COMMSFPGA_H_
#define COMMSFPGA_H_
#include "stm32f3xx_hal.h"
#include "spi.h"

typedef struct wheelVelocityPacket{
	int32_t velocityWheel1;
	int32_t velocityWheel2;
	int32_t velocityWheel3;
	int32_t velocityWheel4;
	uint8_t enablesWheels;
}wheelVelocityPacket;

void setWheelEnable(wheelVelocityPacket* packetToBoi, uint8_t wheel);

void setWheelDisable(wheelVelocityPacket* packetToBoi, uint8_t wheel);

void sendReceivePacket(SPI_HandleTypeDef* spiHandle, wheelVelocityPacket* packetToBoi, wheelVelocityPacket* packetBack);

//void enableWheel(SPI_HandleTypeDef* spiHandle, wheelVelocityPacket* packetToBoi, uint8_t wheel);
//
//void disableWheel(SPI_HandleTypeDef* spiHandle, wheelVelocityPacket* packetToBoi, uint8_t wheel);

void csLow();

void csHigh();

void reverseByte(uint8_t* byteIn);

#endif /* COMMSFPGA_H_ */
