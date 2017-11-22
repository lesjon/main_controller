/*
 * speedcalc.c
 *
 *  Created on: 7 Nov 2016
 *      Author: rik
 */

#include "stm32f3xx_hal.h"
#include "speedcalc.h"
#include "commsfpga.h"
#include <math.h>
#include <myNRF24.h>
#define _POLE_PAIRS 8						// Amount of EL rotations for 1 mechanical rotation
#define _HALL_C_PER_ELROT 6			// Amount of changes in hall effect sensors for 1 EL rotation
#define _MECH_REDUCTION 32/15 			// Amount of mechanical gear reduction 1:M
#define _FPGA_CLOCK_F 1000000	// Clock frequency of the FPGA used to count the time for a hall effect change
//#define _a1  -1.0471975512f
//#define _a3  1.0471975512f
//#define _a4  2.09439510239f
//#define _a2  -2.09439510239f
const float _a0 = 60 * 3.1415/180.0; //240
const float _a1 = 120 * 3.1415/180.0; //300
const float _a2 = 240  * 3.1415/180.0; //60
const float _a3 = 300 * 3.1415/180.0; //120
#define _R   0.09f
#define _r   0.0275f
#define PI 3.1415f

void splitVector(float magnitude, float direction, float* xComponent, float* yComponent){
	magnitude = magnitude / 1000; //from mm/s to m/s;

	direction = direction * (2*M_PI/512);;
	float cosDir = cos(direction);
	float sinDir = sin(direction);


	*xComponent = cosDir*magnitude;
	*yComponent = sinDir*magnitude;
}

//calculate speeds of each individual motor and output to a wheelVelocityPacket
void calcMotorRaw(wheelVelocityPacket* calcPacket, float* prevWheelCommand, float vx, float vy, uint16_t w, uint8_t rotDir){
	float wRadPerSec;

	float accStep = 1.0;
	float accSlowlyUntil = 1.0;

	float wheelScalar = 1/_r;
	float angularComponent;
	float speedmotor[4];
	int rotSign;

	if(rotDir != 0){
		rotSign = -1;
	}
	else{
		rotSign = 1;
	}

	wRadPerSec = (w/180.0)*PI;
	angularComponent = rotSign*_R*wRadPerSec;

	speedmotor[0] = (-cos(_a0)*vy * 1.4 + sin(_a0)*vx + angularComponent)*wheelScalar;
	speedmotor[1] = (-cos(_a1)*vy * 1.4 + sin(_a1)*vx + angularComponent)*wheelScalar;
	speedmotor[2] = (-cos(_a2)*vy * 1.4 + sin(_a2)*vx + angularComponent)*wheelScalar;
	speedmotor[3] = (-cos(_a3)*vy * 1.4 + sin(_a3)*vx + angularComponent)*wheelScalar;


	calcPacket->velocityWheel1=calcFPGAFromRPS(speedmotor[0] / (2*PI));
	calcPacket->velocityWheel2=calcFPGAFromRPS(speedmotor[1] / (2*PI));
	calcPacket->velocityWheel3=calcFPGAFromRPS(speedmotor[2] / (2*PI));
    calcPacket->velocityWheel4=calcFPGAFromRPS(speedmotor[3] / (2*PI));


}


void calcMotorSpeed (dataPacket *dataStruct, wheelVelocityPacket *packetSpeed, float *prevWheelCommand){

	float xSpeed=0; //positive x = moving forward
	float ySpeed=0; //positive y = moving to the right

	//split a vector in a x and y component, based on the magnitude and direction
	splitVector((float)dataStruct->robotVelocity, (float)dataStruct->movingDirection, &xSpeed, &ySpeed);
	//calculate the speed of each individual motor
	calcMotorRaw(packetSpeed, prevWheelCommand, xSpeed, ySpeed,  dataStruct->angularVelocity, dataStruct->rotationDirection);
}


/*
float calcRPSFromFGPA(int32_t iFPGASpeed){
	return ((_FPGA_CLOCK_F )/(_POLE_PAIRS * _HALL_C_PER_ELROT * iFPGASpeed * _MECH_REDUCTION));
};

float calcRPMFromFGPA(int32_t iFPGASpeed){
	return ((60 * _FPGA_CLOCK_F )/(_POLE_PAIRS * _HALL_C_PER_ELROT * iFPGASpeed * _MECH_REDUCTION));
};*/

//this function calculates FPGA-units from rounds per second
//FPGA unit: clock-cycles per HAL-effect update (the FPGA counts the number of clock-cycles between HAL-effect updates, and uses it as it's speed)
//A motor has 6 HAL-effect updates per electric rotation (google working of brushless motor if this is not understood)
//Our motor has 8 pairs of coils, so there are 8 electronic rotations per rotation of the shaft
//Lastly, because of a gears, 3 rotations of the shaft results in 1 rotation of the wheel
//min wheel speed = 1000000 FPGA-units
int32_t calcFPGAFromRPS(float RPS){
	float constant = _FPGA_CLOCK_F/(_POLE_PAIRS * _HALL_C_PER_ELROT * _MECH_REDUCTION);
	int returning = (int)constant/RPS;
	float absReturning = abs(returning);
	if(absReturning > 1000000){
		returning = 0;
	}
    return returning;
};

/*
int32_t calcFPGAFromRPM(float RPM){
    return (abs((_FPGA_CLOCK_F)/(_POLE_PAIRS * _HALL_C_PER_ELROT * _MECH_REDUCTION * RPM/60))>1000000 ? 0 : (_FPGA_CLOCK_F)/(_POLE_PAIRS * _HALL_C_PER_ELROT * _MECH_REDUCTION * RPM/60));
};*/

