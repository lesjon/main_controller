/*
 * commsfpga.c
 *
 *  Created on: 1 Nov 2016
 *      Author: rik
 */


#include <myNRF24.h>
#include "commsfpga.h"
#include "PuttyInterface.h"


void setWheelEnable(wheelVelocityPacket* packetToBoi, uint8_t wheel){
	packetToBoi->enablesWheels |=(uint8_t)0x100>>(wheel);
}

void setWheelDisable(wheelVelocityPacket* packetToBoi, uint8_t wheel){
	packetToBoi->enablesWheels &=~(uint8_t)(1<<(9-wheel));
}

void sendReceivePacket(SPI_HandleTypeDef* spiHandle, wheelVelocityPacket* packetToBoi, wheelVelocityPacket* packetBack){
	uint8_t rawbytessend[12], rawbytesreceived[12];
	for (int i=11;i>=0;i--){
		rawbytessend[i]=0;
	}
	if (packetToBoi->velocityWheel1<0){
		rawbytessend[0]|= 0x80;
	}
		rawbytessend[0]|= (uint8_t)((packetToBoi->velocityWheel1&0x000FE000)>>13);
		rawbytessend[1]|= (uint8_t)((packetToBoi->velocityWheel1&0x00001FE0)>>5);
		rawbytessend[2]|= (uint8_t)((packetToBoi->velocityWheel1&0x0000001F)<<3);
	if (packetToBoi->velocityWheel2<0){
		rawbytessend[2]|= 0x04;
	}
		rawbytessend[2]|= (uint8_t)((packetToBoi->velocityWheel2&0x000C0000)>>18);
		rawbytessend[3]|= (uint8_t)((packetToBoi->velocityWheel2&0x0003FC00)>>10);
		rawbytessend[4]|= (uint8_t)((packetToBoi->velocityWheel2&0x000003FC)>>2);
		rawbytessend[5]|= (uint8_t)((packetToBoi->velocityWheel2&0x00000003)<<6);
	if (packetToBoi->velocityWheel3<0){
		rawbytessend[5]|= 0x20;
	}
		rawbytessend[5]|= ((packetToBoi->velocityWheel3&0x000F8000)>>15);
		rawbytessend[6]|= ((packetToBoi->velocityWheel3&0x00007F80)>>7);
		rawbytessend[7]|= ((packetToBoi->velocityWheel3&0x0000007F)<<1);
	if (packetToBoi->velocityWheel4<0){
		rawbytessend[7]|= 0x01;
	}
		rawbytessend[8]|= ((packetToBoi->velocityWheel4&0x000FF000)>>12);
		rawbytessend[9]|= ((packetToBoi->velocityWheel4&0x00000FF0)>>4);
		rawbytessend[10]|= ((packetToBoi->velocityWheel4&0x0000000F)<<4);
		rawbytessend[10]|=((packetToBoi->enablesWheels&0xF0)>>4);
		rawbytessend[11]=42; //lol

		for(int i = 0; i < 12; i++){
			reverseByte(&rawbytessend[i]);
		}

		csLow();

		for (int i=11; i>=0; i--){

			HAL_SPI_TransmitReceive(spiHandle, &rawbytessend[i], &rawbytesreceived[i], 1, 100);
			reverseByte(&rawbytesreceived[i]);
			//sprintf(smallStrBuffer, "%x ", rawbytesreceived[i]);
			//TextOut(smallStrBuffer);
		}
		//TextOut("\n");

		csHigh();
		if((rawbytesreceived[0]&0b10000000) != 0){ // Speed1 = negative
		    packetBack->velocityWheel1 = - (int32_t)((~( (uint32_t)((rawbytesreceived[2]&0b11111000)>>3) | ((uint32_t)rawbytesreceived[1]<<5) | ((uint32_t)(rawbytesreceived[0]&0b01111111)<<13) ))&0xFFFFF) -1;
		  }else{
		    packetBack->velocityWheel1 = (uint32_t)((rawbytesreceived[2]&0b11111000)>>3) | ((uint32_t)rawbytesreceived[1]<<5) | ((uint32_t)(rawbytesreceived[0]&0b01111111)<<13);
		  }
		  if((rawbytesreceived[2]&0b00000100) != 0){ // Speed2 = negative
			  packetBack->velocityWheel2 = - (int32_t)((~( ((uint32_t)((rawbytesreceived[5]&0b11000000)>>6) | ((uint32_t)rawbytesreceived[4]<<2) | ((uint32_t)rawbytesreceived[3]<<10) | ((uint32_t)(rawbytesreceived[2]&0b00000011)<<18)) ))&0xFFFFF) -1;
		  }else{
			  packetBack->velocityWheel2 = (uint32_t)((rawbytesreceived[5]&0b11000000)>>6) | ((uint32_t)rawbytesreceived[4]<<2) | ((uint32_t)rawbytesreceived[3]<<10) | ((uint32_t)(rawbytesreceived[2]&0b00000011)<<18);
		  }
		  if((rawbytesreceived[5]&0b00100000) != 0){ // Speed3 = negative
			  packetBack->velocityWheel3 = - (int32_t)((~( ((rawbytesreceived[7]&0b11111110)>>1) | ((uint32_t)rawbytesreceived[6]<<7) | ((uint32_t)(rawbytesreceived[5]&0b00011111)<<15) ))&0xFFFFF) -1;
		  }else{
			  packetBack->velocityWheel3 = (uint32_t)((rawbytesreceived[7]&0b11111110)>>1) | ((uint32_t)rawbytesreceived[6]<<7) | ((uint32_t)(rawbytesreceived[5]&0b00011111)<<15);
		  }
		  if((rawbytesreceived[7]&0b00000001) != 0){ // Speed4 = negative
			  packetBack->velocityWheel4 = - (int32_t)((~( ((rawbytesreceived[10]&0b11110000)>>4) | ((uint32_t)rawbytesreceived[9]<<4) | ((uint32_t)(rawbytesreceived[8])<<12) ))&0xFFFFF) -1;
		  }else{
			  packetBack->velocityWheel4 = (uint32_t)((rawbytesreceived[10]&0b11110000)>>4) | ((uint32_t)rawbytesreceived[9]<<4) | ((uint32_t)(rawbytesreceived[8])<<12);
		  }
}

//void enableWheel(SPI_HandleTypeDef* spiHandle, wheelVelocityPacket* packetToBoi, uint8_t wheel){
//	setWheelEnable(packetToBoi, wheel);
//	sendPacket(spiHandle, packetToBoi);
//}
//
//void disableWheel(SPI_HandleTypeDef* spiHandle, wheelVelocityPacket* packetToBoi, uint8_t wheel){
//	setWheelDisable(packetToBoi, wheel);
//	sendPacket(spiHandle, packetToBoi);
//}

// (SPI3_CS_GPIO_Port, SPI3_CS_Pin) to low
void csLow(){
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
}

void csHigh(){
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}

void reverseByte(uint8_t* byteIn){
	uint8_t byteOut = 0;

	byteOut += (*byteIn & 0x80) >> 7;
	byteOut += (*byteIn & 0x40) >> 5;
	byteOut += (*byteIn & 0x20) >> 3;
	byteOut += (*byteIn & 0x10) >> 1;
	byteOut += (*byteIn & 0x08) << 1;
	byteOut += (*byteIn & 0x04) << 3;
	byteOut += (*byteIn & 0x02) << 5;
	byteOut += (*byteIn & 0x01) << 7;

	*byteIn = byteOut;

}
