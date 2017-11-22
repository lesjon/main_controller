/*
 * MTiControl.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Leon
 */
#include "MTiControl.h"
#include "PuttyInterface.h"
#include <string.h>

uint8_t aRxBuffer[MTI_READ_PER_BYTE ? 2 : MTDATA2_MESSAGE_SIZE];
struct XbusParser * XBParser;
uint8_t HAL_UART_TxCpltCallback_flag = 0;
uint8_t HAL_UART_RxCpltCallback_flag = 0;
uint8_t HAL_UART_ErrorCallback_flag = 0;
uint8_t bytes_received[MAX_RAW_MESSAGE_SIZE];
uint16_t bytes_received_cnt = 0;

//also extern
uint8_t cplt_mess_stored_flag = 0;
struct XbusMessage* ReceivedMessageStorage;

static void XBP_handleMessage(struct XbusMessage const* message);
static void* XBP_allocateBuffer(size_t bufSize);
static void XBP_deallocateBuffer(void const* buffer);
static HAL_StatusTypeDef Usart3ReceiveMessage_IT(uint8_t* message, uint16_t size);
static void StoreReceivedBytes();

void CheckWhatNeedsToBeDone(){
	if(HAL_UART_TxCpltCallback_flag){
		HAL_UART_TxCpltCallback_flag = 0;
	}
	if(HAL_UART_RxCpltCallback_flag != 0){
		HAL_UART_RxCpltCallback_flag = 0;
		StoreReceivedBytes();
	}
	if(HAL_UART_ErrorCallback_flag != 0){
		HAL_UART_ErrorCallback_flag = 0;
	}
}
// Everytime a byte is received over usart this function will be called and store/parse it in the xbusparser.
static void StoreReceivedBytes(){
	static uint16_t bytes_parsed = 0;
	while(bytes_parsed < bytes_received_cnt){
		XbusParser_parseByte(XBParser, bytes_received[bytes_parsed++]);
	}
	if(cplt_mess_stored_flag){
		uprintf("bytes_parsed = [%u]\n\r", bytes_parsed);
		bytes_parsed = 0;
		bytes_received_cnt = 0;
		return;
	}
	Usart3ReceiveMessage_IT((uint8_t *)aRxBuffer, 1);
}
HAL_StatusTypeDef Usart3ReceiveMessage_IT(uint8_t* message, uint16_t size){
	HAL_StatusTypeDef return_value;
	return_value = HAL_UART_Receive_IT(&MTi_huartx, message, size);

	return return_value;
}
// Callback is called when the HAL_Uart application is finished transmitting its bytes
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	// Prevent unused argument(s) compilation warning
	UNUSED(huart);

	HAL_UART_TxCpltCallback_flag = 1;
}
// Callback is called when the HAL_Uart received its wanted amount of bytes
void MTi_HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	for(uint8_t i= 0; i < (MTI_READ_PER_BYTE ? 1 : MTDATA2_MESSAGE_SIZE); i++ ){
		bytes_received[bytes_received_cnt++] = aRxBuffer[i];
	}
	HAL_UART_RxCpltCallback_flag = 1;
}
// Callback is called when the HAL_Uart application returns an error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);

	HAL_UART_ErrorCallback_flag = 1;
}
// An xbusmessage is formatted and sent over usart
void SendXbusMessage(struct XbusMessage XbusMessage){
	uint8_t raw[128];
	size_t XbusMes_size =  XbusMessage_format(raw, (struct XbusMessage const*)&XbusMessage, XLLF_Uart);
	char str_buf[32];
	for(uint8_t i = 0; i < XbusMes_size; i++){
		sprintf(str_buf, "[%02x]",raw[i]);
		TextOut(str_buf);
	}
	TextOut("\n\r");
	HAL_UART_Transmit(&MTi_huartx, raw, XbusMes_size, 0xFFFF);
}
//This function sends the wakeUpAck message.
//param  None
//retval 0 if sent -1 is error, 1 is timout, busy or smth else
void SendWakeUpAck(){
	struct XbusMessage XbusMes = {	.mid = XMID_WakeUpAck,
									.length =  0,
									.data = NULL};
	SendXbusMessage(XbusMes);
}
// Wait till a certain message type is received from MTi over usart
int WaitForAck(enum XsMessageId XMID){
	uint32_t cnt = 0;
	cplt_mess_stored_flag = 0;
	Usart3ReceiveMessage_IT((uint8_t *)aRxBuffer, 1);
	ReceivedMessageStorage->mid = 0;
	while(cnt < 0xFFFFFF && !(ReceivedMessageStorage->mid == XMID)){
		cnt++;
		if(HAL_UART_RxCpltCallback_flag != 0){
			HAL_UART_RxCpltCallback_flag = 0;
			StoreReceivedBytes();
		}
	}
	if(cnt < 0xFFFFFF){
		uprintf("ReceivedMessageStorage->mid = [%02x]\n\r", ReceivedMessageStorage->mid);
		return 1;
	}
	return 0;
}
// Initialize controlling the MTi device
void MTi_Init(){
	ReceivedMessageStorage = malloc(MAX_RAW_MESSAGE_SIZE);// Reserve memory to store the longest possible message

	//HAL_GPIO_WritePin(MT_RST_GPIO_Port, MT_RST_Pin, MTI_NO_RESET);// set the MTi in reset state
	struct XbusParserCallback XBP_callback = {};// Create a struct to contain the callback functions
	XBP_callback.handleMessage = XBP_handleMessage;
	XBP_callback.allocateBuffer = XBP_allocateBuffer;
	XBP_callback.deallocateBuffer = XBP_deallocateBuffer;
	XBParser = XbusParser_create(&XBP_callback);// Create an XBus parser
}

// Start reading a new message
// if cancel_previous, the current running receive operation is cancelled
void ReadNewMessage(uint8_t cancel_previous){
	if(cancel_previous){
		HAL_UART_AbortReceive(&MTi_huartx);
	}

	if(MTI_READ_PER_BYTE){
		Usart3ReceiveMessage_IT((uint8_t *)aRxBuffer, 1);
	}else {
		Usart3ReceiveMessage_IT((uint8_t *)aRxBuffer, MTDATA2_MESSAGE_SIZE);
	}
}
// When a complete message is received from the device this function will process it

void MtiReset(){
	//HAL_GPIO_WritePin(MT_RST_GPIO_Port, MT_RST_Pin, 0);
}
void MtiSoftReset(){
	struct XbusMessage mess = {XMID_Reset};
	SendXbusMessage(mess);
}
void CancelMtiOperation(){
	HAL_UART_Abort(&MTi_huartx);
	MtiReset();
}
// XBP_handleMessage is called when the xbusparser is done parsing one message
static void XBP_handleMessage(struct XbusMessage const* message){
	cplt_mess_stored_flag = 1;
	memcpy(ReceivedMessageStorage, message, MAX_RAW_MESSAGE_SIZE);
}
static void* XBP_allocateBuffer(size_t bufSize){
	return bufSize < MAX_RAW_MESSAGE_SIZE? malloc(bufSize) : NULL;
}
static void XBP_deallocateBuffer(void const* buffer){
	free((uint8_t(*)[MAX_RAW_MESSAGE_SIZE])buffer);
}
void DeallocateMem(){
	TheAlligator(XBParser);
}

MTiConfiguration RequestMTiConfiguration(){
	MTiConfiguration config = {};
	uint8_t config_message_size = 118+5;
	uint8_t * rec_mess = malloc(config_message_size);
	struct XbusMessage  mess = {.mid = XMID_ReqConfiguration,
								.data = NULL,
								.length = 0};
	SendXbusMessage(mess);
	Usart3ReceiveMessage_IT(rec_mess, config_message_size);
	XbusParser_parseBuffer(XBParser, rec_mess, config_message_size);
	free(rec_mess);
	uint8_t const* dptr = ReceivedMessageStorage->data;
	sprintf(smallStrBuffer, "mid = %02x, length = %u\n\r", ReceivedMessageStorage->mid, ReceivedMessageStorage->length );
	TextOut(smallStrBuffer);
	if(!(XMID_Configuration == ReceivedMessageStorage->mid)){
		return config;
	}
	for(uint8_t i = 0; i < ReceivedMessageStorage->length; i++){
		sprintf(smallStrBuffer, "%u:[%02x]", i, *(dptr + i));
		TextOut(smallStrBuffer);
	}
	TextOut("\n\r");
	dptr = XbusUtility_readU32(&config.Master_device_ID, dptr);
	dptr = XbusUtility_readU16(&config.Sampling_period, dptr);
	dptr = XbusUtility_readU16(&config.Output_skip_factor, dptr);
	dptr = XbusUtility_readU16(&config.Syncin_settings_Mode, dptr);
	dptr = XbusUtility_readU16(&config.Syncin_settings_SkipvFactor, dptr);
	dptr = XbusUtility_readU32(&config.Syncin_settings_Offset, dptr);
	for(uint8_t i = 0; i < 8; i++){
		dptr = XbusUtility_readU8(&config.Date[i], dptr);
	}
	for(uint8_t i = 0; i < 8; i++){
		dptr = XbusUtility_readU8(&config.Time[i], dptr);
	}
	for(uint8_t i = 0; i < 32; i++){
		dptr = XbusUtility_readU8(&config.Reserved_host[i], dptr);
	}
	for(uint8_t i = 0; i < 32; i++){
		dptr = XbusUtility_readU8(&config.Reserved_client[i], dptr);
	}
	dptr = XbusUtility_readU16(&config.Number_of_devices, dptr);
	dptr = XbusUtility_readU32(&config.Device_ID, dptr);
	dptr = XbusUtility_readU16(&config.Data_length_MTData2_message, dptr);
	dptr = XbusUtility_readU16(&config.Output_mode, dptr);
	dptr = XbusUtility_readU32(&config.Output_settings, dptr);
	for(uint8_t i = 0; i < 8; i++){
		dptr = XbusUtility_readU8(&config.Reserved[i], dptr);
	}
	return config;
}
