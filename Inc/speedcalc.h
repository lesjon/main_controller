/*
 * speedcalc.h
 *
 *  Created on: 7 Nov 2016
 *      Author: rik
 */

#ifndef SPEEDCALC_H_
#define SPEEDCALC_H_
#include <myNRF24.h>
#include "stm32f3xx_hal.h"
#include "commsfpga.h"

// splits a polar vector into a Carthesian one
void splitVector(float magnitude, float direction, float* xComponent, float* yComponent);

/*calculates the speeds of the motor gives a carthesian vector and a a rotational velocity
* the function updates the values in the velocity struct
*/
void calcMotorRaw(wheelVelocityPacket* calcpacket, float* prevWheelCommand, float vx, float vy, uint16_t w, uint8_t rotDir);

/* Calculates the speed of the motor given the datapacket of the PC
* the function updates the values in the velocity struct
*/
void calcMotorSpeed(dataPacket *datastruct, wheelVelocityPacket *PacketSpeed, float *prevWheelCommand);

/* Calculates the speed of the motor given the Stefans method of the PC
* the function updates the values in the velocity struct
*/
void calcMotorStefan(dataPacket *dataStruct, wheelVelocityPacket *PacketSpeed);
// FPGAspeed is the amount of clock cycles between 2 commutations of the motor
// maximum is 1 or -1 minimum speed is 1000000

/* calculates the rotations per seconds from the FPGA values
* input is the FPGA value, output is the RPS
*/
float calcRPSFromFGPA(int32_t iFPGASpeed);

/* calculates the rotations per minute from the FPGA values
* input is the FPGA value, output is the RPM
*/
float calcRPMFromFGPA(int32_t iFPGASpeed);

/* calculates the FPGA values from the rotations wanted
* input is the RPS value, output is the FPGA
*/
int32_t calcFPGAFromRPS(float RPS);

/* calculates the FPGA values from the rotations wanted
* input is the RPS value, output is the FPGA
*/
int32_t calcFPGAFromRPM(float RPM);


#endif /* SPEEDCALC_H_ */
