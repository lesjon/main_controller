/*
 * MTiControl.h
 *
 *  Created on: Sep 25, 2017
 *      Author: Leon
 */

#ifndef MTICONTROL_H_
#define MTICONTROL_H_

#include <stdlib.h>
#include "main.h"
#include "usart.h"
#include "stdint.h"
#include "xbusmessage.h"
#include "xbusparser.h"
#include "xbusutility.h"
#include "xsdeviceid.h"

#define MAX_RAW_MESSAGE_SIZE 	2055
#define MTi_huartx				huart2
#define MTI_NO_RESET			1 // 0 to start the mti in reset state, 1 to let the mti start itself immediatly
#define MTDATA2_MESSAGE_SIZE	40 //
#define MTI_READ_PER_BYTE		0 // 0 to read a per whole mtdata2 message (length assumed MTDATA2_MESSAGE_SIZE bytes)

typedef struct {
	uint32_t Master_device_ID;
	uint16_t Sampling_period;
	uint16_t Output_skip_factor;
	uint16_t Syncin_settings_Mode;
	uint16_t Syncin_settings_SkipvFactor;
	uint32_t Syncin_settings_Offset;
	uint8_t Date[8];// format YYYYMMDD (can be set by host)
	uint8_t Time[8];// format HHMMSSHH (can be set by host)
	uint8_t Reserved_host[32];
	uint8_t Reserved_client[32];
	uint16_t Number_of_devices;// ( = 1 (0x0001))
	uint32_t Device_ID;// (same as master device ID)
	uint16_t Data_length_MTData2_message;
	uint16_t Output_mode;
	uint32_t Output_settings;
	uint8_t Reserved[8];
} MTiConfiguration;

extern uint8_t cplt_mess_stored_flag;
extern struct XbusMessage* ReceivedMessageStorage;

void MTi_Init();
void CancelMtiOperation();
void MtiReset();
void SendWakeUpAck();
void MTi_HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int WaitForAck(enum XsMessageId XMID);
void SendXbusMessage(struct XbusMessage XbusMessage);
void ReadNewMessage(uint8_t cancel_previous);
void CheckWhatNeedsToBeDone();
void DeallocateMem();
MTiConfiguration RequestMTiConfiguration();
void MtiSoftReset();

#endif /* MTICONTROL_H_ */
