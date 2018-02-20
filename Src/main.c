/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "myNRF24.h"
#include "commsfpga.h"
#include "speedcalc.h"
#include "MTiControl.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t address = 7 ;
uint8_t freqChannel = 78;
bool stop_after_message_complete = true;
bool message_handled_flag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void shoot(uint8_t intensity, uint8_t chipper, uint8_t id);
void HandleMessage();
void MTiErrorHandler(struct XbusMessage const* message);
void printMessageData(struct XbusMessage const* message);
void UintToLeds(uint8_t n);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t triedSystemClock = 0;
static void trySystemClock(){
	if(triedSystemClock == 0){
		SystemClock_Config();
		triedSystemClock = 1;
	}
}
#define SystemClock_Config trySystemClock

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  #undef SystemClock_Config
  UintToLeds(address);
  HAL_Delay(1000);


  //init dribbler
  TIM_OC_InitTypeDef dribbler;
  int kickprev=0;

  dribbler.OCMode = TIM_OCMODE_PWM2;
  dribbler.Pulse = 0;
  dribbler.OCPolarity = TIM_OCPOLARITY_HIGH;
  dribbler.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &dribbler, TIM_CHANNEL_2) != HAL_OK){
		 Error_Handler();
  }

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  nssHigh(&hspi2);

  initRobo(&hspi2, freqChannel, address);
  dataPacket dataStruct;
  wheelVelocityPacket wheely;
  wheelVelocityPacket backWheely;
  float prevWheelCommand[4];

  wheely.velocityWheel1 = 0;
  wheely.velocityWheel2 = 0;
  wheely.velocityWheel3 = 0;
  wheely.velocityWheel4 = 0;
  int stopCnt = 0;

  int kickPrevCnt = 1000;

  UintToLeds(0);// preferably a power of 2

  volatile uint8_t test;

  test = readReg(&hspi2, 0x05);

  while (1)
  {
	  if(irqRead(&hspi2)){
		  HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, 1);
		  stopCnt = 0;
		  roboCallback(&hspi2, &dataStruct);
		  if(dataStruct.robotID == address){
			  HAL_GPIO_TogglePin(led3_GPIO_Port, led3_Pin);
			  calcMotorSpeed(&dataStruct, &wheely, &prevWheelCommand);

			  //dribbler
			  dribbler.Pulse=125*dataStruct.driblerSpeed;

			  if (HAL_TIM_PWM_ConfigChannel(&htim2, &dribbler, TIM_CHANNEL_2) != HAL_OK){
				  Error_Handler();
			  }

			  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

			  //kicker
			  if (dataStruct.kickForce != 0){
				  if(kickPrevCnt >= 999){
					  kickPrevCnt = 0;
					  shoot(dataStruct.kickForce, dataStruct.chipper, address);
					  dataStruct.kickForce = 0;
				  }
			  }
		  }

	  }



	  sendReceivePacket(&hspi3, &wheely, &backWheely);

	  HAL_Delay(1);

	  if(stopCnt > 250){
		  stopCnt = 0;
		  wheely.velocityWheel1 = 0;
		  wheely.velocityWheel2 = 0;
		  wheely.velocityWheel3 = 0;
		  wheely.velocityWheel4 = 0;
	  }
	  else{
		  stopCnt++;
	  }
	  UintToLeds(stopCnt & (16+32));


	  if(kickPrevCnt < 1000){
		  kickPrevCnt++;
	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void shoot(uint8_t intensity, uint8_t chipper, uint8_t id){


	/*HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, GPIO_PIN_RESET);*/




	int period = 0 + intensity*25;

	htim1.Init.Period = period;

	HAL_TIM_Base_Init(&htim1);

	//DEBUG
	//id == 10 because kicker 10 is broken
	if(chipper == 0 && id != 0){
		HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(chipper_GPIO_Port, chipper_Pin, GPIO_PIN_SET);
	}
	HAL_TIM_Base_Start_IT(&htim1);


}
void HandleMessage(){
	if(ReceivedMessageStorage->mid == XMID_Error){
		MTiErrorHandler(ReceivedMessageStorage);
	}else if(ReceivedMessageStorage->mid == XMID_MTData2){
		printMessageData(ReceivedMessageStorage);
	}
	message_handled_flag = 1;
	DeallocateMem();
}

void MTiErrorHandler(struct XbusMessage const* message){
	CancelMtiOperation();
	uint16_t i = 0;
	uint8_t j = 0;
	while(1){
		if(!(++i)){
			if(!(++j)){
				  HAL_GPIO_TogglePin(led1_GPIO_Port, led1_Pin);
				  HAL_GPIO_TogglePin(led2_GPIO_Port, led2_Pin);
				  HAL_GPIO_TogglePin(led3_GPIO_Port, led3_Pin);
				  HAL_GPIO_TogglePin(led4_GPIO_Port, led4_Pin);
				  HAL_GPIO_TogglePin(led5_GPIO_Port, led5_Pin);
				  HAL_GPIO_TogglePin(led6_GPIO_Port, led6_Pin);
			}
		}
	}

}
void printMessageData(struct XbusMessage const* message){
	if (!message)
		return;
	uint16_t counter;
	if (XbusMessage_getDataItem(&counter, XDI_PacketCounter, message))
	{
	}
	uint32_t SampleTimeFine;
	if (XbusMessage_getDataItem(&SampleTimeFine, XDI_SampleTimeFine, message))
	{

	}
	float ori[4];
	if (XbusMessage_getDataItem(ori, XDI_Quaternion, message))
	{
	}
	float angles[3];
	if (XbusMessage_getDataItem(angles, XDI_EulerAngles, message))
	{
	}
	float delta_v[3];
	if (XbusMessage_getDataItem(delta_v, XDI_DeltaV, message))
	{
	}
	float acc[3];
	if (XbusMessage_getDataItem(acc, XDI_Acceleration, message))
	{
		UintToLeds((uint8_t)(acc[0]*3));
	}
	float gyr[3];
	if (XbusMessage_getDataItem(gyr, XDI_RateOfTurn, message))
	{
	}
	float delta_q[4];
	if (XbusMessage_getDataItem(delta_q, XDI_Quaternion, message))
	{
	}
	float mag[3];
	if (XbusMessage_getDataItem(mag, XDI_MagneticField, message))
	{
	}
	uint32_t status;
	if (XbusMessage_getDataItem(&status, XDI_StatusWord, message))
	{

	}
}
void UintToLeds(uint8_t n){
	  HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, n & 1);
	  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, n & 2);
	  HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, n & 4);
	  HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, n & 8);
	  HAL_GPIO_WritePin(led5_GPIO_Port, led5_Pin, n & 16);
	  HAL_GPIO_WritePin(led6_GPIO_Port, led6_Pin, n & 32);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
