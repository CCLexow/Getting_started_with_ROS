
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

osThreadId Main_TaskHandle;
osThreadId COM_TaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint8_t received_data[100];
extern uint32_t received_data_size;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
void MainTaskWork(void const * argument);
void COMTaskWork(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Main_Task */
  osThreadDef(Main_Task, MainTaskWork, osPriorityNormal, 0, 1024);
  Main_TaskHandle = osThreadCreate(osThread(Main_Task), NULL);

  /* definition and creation of COM_Task */
  osThreadDef(COM_Task, COMTaskWork, osPriorityNormal, 0, 512);
  COM_TaskHandle = osThreadCreate(osThread(COM_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SYS_LED_Pin */
  GPIO_InitStruct.Pin = SYS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_LightBarrier_Pin */
  GPIO_InitStruct.Pin = IN_LightBarrier_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_LightBarrier_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

typedef enum{
	eMain_TxStopped = 0,
	eMain_TxContinuous,
	eMain_TxNSamples
}T_Main_Tx_States;
T_Main_Tx_States xMainTxState = eMain_TxContinuous;

uint16_t u16TxSampleCnt;

#define  SERIAL_CMD_MAX_LENGTH	20
typedef struct USB_RX_Command_Struct
{
	uint8_t u08CommandReceived;
	uint8_t u08WriteIdx;
	uint8_t u08FreeBytes;
	uint8_t au08CommandBuffer[SERIAL_CMD_MAX_LENGTH];
}T_USB_RX_Command;
T_USB_RX_Command xUartRxCmd;


void FlushRxCmd(void)
{
	xUartRxCmd.u08CommandReceived = 0;
	xUartRxCmd.u08WriteIdx = 0;
	for(uint8_t u08Idx=0; u08Idx < sizeof(xUartRxCmd.au08CommandBuffer); u08Idx++)
	{
	  xUartRxCmd.au08CommandBuffer[u08Idx] = 0;
	}
	xUartRxCmd.u08FreeBytes = SERIAL_CMD_MAX_LENGTH;
}

void AnalyseRxCmd(void)
{
	char * cmd_perform_reduced_scan = "RSCAN %d";
	char * cmd_perform_complete_scan = "CSCAN";
	char * cmd_sample_distance_sensor = "DISTCONV";
	char * cmd_sample_distance_sensor_N = "DISTNCONV %d";
	char * cmd_stop_sample_distance_sensor = "STOPDISTCONV";

	int32_t i32Param = 0;

	/* analyse received command	*/
	//iRetVal = sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_Trig_level,&i32Param);
	//if(0 < iRetVal)
	if(0 < sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_perform_reduced_scan,&i32Param))
	{
		/* perform reduced resolution scan with 360° / i32Param degree steps*/

	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_perform_complete_scan))
	{
		/* perform a full resolution scan */

	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_sample_distance_sensor))
	{
		/* request to stop reflow process */
		xMainTxState = eMain_TxContinuous;

	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_stop_sample_distance_sensor))
	{
		/* request to stop reflow process */
		xMainTxState = eMain_TxStopped;

	}
	else if(0 < sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_sample_distance_sensor_N, &i32Param))
	{
		if(0 < i32Param){
			if(UINT16_MAX >= i32Param)
			{
				u16TxSampleCnt = (uint16_t)i32Param;
				xMainTxState = eMain_TxNSamples;
			}
		}
	}
}

#define DISTSENSE_VOUT_SAMPLING_CNT	8
uint32_t Distance_ConvertSensorOutput(void)
{
	uint8_t u08SampleCount = DISTSENSE_VOUT_SAMPLING_CNT;
	uint32_t u32DistSensorVal = 0;

	HAL_StatusTypeDef xHAL_Status;


	while(u08SampleCount--)
	{
		/* start an a/d conversion */
		xHAL_Status = HAL_ADC_Start(&hadc1);
		/* exit on error*/
		if(HAL_OK != xHAL_Status)
		{
			return 0;
		}
		/* wait for conversion to finish */
		xHAL_Status = HAL_ADC_PollForConversion(&hadc1, 100);
		/* exit on error*/
		if(HAL_OK != xHAL_Status)
		{
			return 0;
		}
		/* get and sum up sampled value*/
		u32DistSensorVal += HAL_ADC_GetValue(&hadc1);
	}
	return u32DistSensorVal/DISTSENSE_VOUT_SAMPLING_CNT;
}

void Print_Sensor_Data(uint32_t u32Data)
{
	uint8_t au08DataOutput[100];
	static uint8_t su08MsgCnt = 0;
	volatile float fltADCVoltage;
	volatile uint16_t u16ADCVoltage;

	if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state)
	{
		fltADCVoltage = u32Data * 3.3f;
		fltADCVoltage /= 4096;
		fltADCVoltage *= 100;
		u16ADCVoltage = (uint16_t)fltADCVoltage;
		sprintf((char *)&au08DataOutput[0], "%d %d %d\n", su08MsgCnt, u32Data,u16ADCVoltage);
		su08MsgCnt++;
		CDC_Transmit_FS(&au08DataOutput[0],strlen(au08DataOutput));
	}
}

/* USER CODE END 4 */

/* MainTaskWork function */
void MainTaskWork(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100; // every 100 ms
	volatile uint32_t u32ADCVal;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
	for(;;)
	{
		/* create fixed frequency task calling */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		HAL_GPIO_TogglePin(SYS_LED_GPIO_Port,SYS_LED_Pin);

		u32ADCVal = Distance_ConvertSensorOutput();

		if(eMain_TxContinuous == xMainTxState)
		{
			Print_Sensor_Data(u32ADCVal);
		}
		else if(eMain_TxNSamples == xMainTxState)
		{
			if(0 < u16TxSampleCnt)
			{
				u16TxSampleCnt--;
				Print_Sensor_Data(u32ADCVal);
			}
			else
			{
				xMainTxState = eMain_TxStopped;
			}
		}

	}
  /* USER CODE END 5 */ 
}

/* COMTaskWork function */
void COMTaskWork(void const * argument)
{
  /* USER CODE BEGIN COMTaskWork */
   HAL_StatusTypeDef xRetVal = HAL_ERROR;
	uint8_t au08RxBuffer[20];
	uint8_t u08Idx;
	volatile uint32_t u32Length;
	volatile int8_t i08USBCDCErrCode;

	/* Infinite loop */
	for(;;)
	{
		osDelay(100);	/* delay for 100 ms */
		/* clear buffer */
		for(u08Idx=0; u08Idx < sizeof(au08RxBuffer); u08Idx++)
		{
			au08RxBuffer[u08Idx] = 0;
		}
		/* check for incoming bytes on UART */
		//xRetVal = HAL_UART_Receive(&huart2,(uint8_t *)&au08RxBuffer[0],sizeof(au08RxBuffer),2);
		//if((HAL_OK == xRetVal) || (HAL_TIMEOUT == xRetVal))
		if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state)
		{
			//i08USBCDCErrCode = CDC_Receive_FS(&au08RxBuffer[0], &u32Length);
			//if(USBD_OK == i08USBCDCErrCode)
			if(0 < received_data_size)
			{
				// copy input into command
				//for(u08Idx=0; u08Idx < sizeof(au08RxBuffer); u08Idx++)
				u08Idx = 0;
				while((received_data_size--) && (100 > u08Idx))
				{
					if('\r' == received_data[u08Idx])
					{
						xUartRxCmd.u08CommandReceived = 1;
						break;
					}
					else if(0 == received_data[u08Idx])
					{
						/* all received bytes processed */
						break;
					}
					else if ('\n' != received_data[u08Idx])	/* ignore new line character */
					{
						/* copy if still space left */
						if (0 < xUartRxCmd.u08FreeBytes)
						{
							xUartRxCmd.au08CommandBuffer[xUartRxCmd.u08WriteIdx++] = received_data[u08Idx];
							xUartRxCmd.u08FreeBytes--;
						}
						else
						{
							/* ERR -> flush */
							FlushRxCmd();
						}
						xUartRxCmd.u08CommandReceived = 0;
					}
					u08Idx++;
				}
			}
			//else if(HAL_ERROR == xRetVal)
			else
			{

			}
			/*else if(HAL_TIMEOUT == xRetVal)
			{

			}*/
		}

		//TODO: es muss noch der USB buffer geflusht werden

		if(1 == xUartRxCmd.u08CommandReceived)
		{
			AnalyseRxCmd();
			FlushRxCmd();
		}
	}
  /* USER CODE END COMTaskWork */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
