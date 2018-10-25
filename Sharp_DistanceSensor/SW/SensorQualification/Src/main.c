
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
//#include "STMDriver.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

osThreadId Main_TaskHandle;
osThreadId COM_TaskHandle;
osThreadId SensorsTaskHandle;
osThreadId SMTD_TaskHandle;
osMessageQId xQMainToSensorHandle;
osMessageQId xQSensorToMainHandle;
osMessageQId xQMainToSTMDHandle;
osMessageQId xQSTMDToMainHandle;
osMessageQId xQMainToCOMHandle;
osMessageQId xQCOMToMainHandle;

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
void SampleSensorsWork(void const * argument);
void SMTDWork(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* definition of commands passed between tasks */
typedef enum eMQ_CommandList
{
	eCMD_None = 0,
	eCMD_Rejected,
	eCMD_Accepted,
	eCMD_PerformHoming,
	eCMD_StaticMeasurement,
	eCMD_FullScanMeasurement,
	eCMD_PartialScanMeasurement,
	eCMD_SampleSensor,
	eCMD_SensorValue,
	eCMD_PerformStep,
	eCMD_Abort
}T_MQ_CommandList;

/* structure to hold commands and their parameters which will be passed between tasks */
typedef struct strMQ_Command
{
	T_MQ_CommandList xMQ_Cmd;
	int32_t i32MQ_Cmd_ParamA;
	int32_t i32MQ_Cmd_ParamB;
}T_MQ_Command;

#define DIST_SENSOR_SAMPLING_CNT	100

#define LINBUFFSIZE		100
typedef struct strBuffer
{
	uint32_t au32Buffer[LINBUFFSIZE];
	uint16_t u16BufferWriteIdx;
	uint16_t u16BufferFree;
}T_LinearBuffer;

T_LinearBuffer xLinearBuffer_1;
T_LinearBuffer xLinearBuffer_2;
T_LinearBuffer * pxLinearBufferActive;
T_LinearBuffer * pxLinearBufferReadyForTx;

void LinearBufferFlush(T_LinearBuffer * pxLinBuff)
{
	/* preset buffer with zeros */
	for(pxLinBuff->u16BufferWriteIdx=0;pxLinBuff->u16BufferWriteIdx<LINBUFFSIZE;pxLinBuff->u16BufferWriteIdx++)
	{
		pxLinBuff->au32Buffer[pxLinBuff->u16BufferWriteIdx] = 0;
	}

	/* preset variables */
	pxLinBuff->u16BufferFree = LINBUFFSIZE;
	pxLinBuff->u16BufferWriteIdx = 0;
}

uint8_t u08LinearBufferPush(T_LinearBuffer * pxLinBuff, uint32_t u32Value)
{
	if(0 < pxLinBuff->u16BufferFree)
	{
		pxLinBuff->au32Buffer[pxLinBuff->u16BufferWriteIdx] = u32Value;
		pxLinBuff->u16BufferWriteIdx++;
		pxLinBuff->u16BufferFree--;
		return 1;
	}
	else
	{
		return 0;
	}
}

void Init_Linear_Buffers(void)
{
	/* flush linear buffers */
	LinearBufferFlush(&xLinearBuffer_1);
	LinearBufferFlush(&xLinearBuffer_2);
	/* assign linear buffer pointer */
	pxLinearBufferActive = & xLinearBuffer_1;
	pxLinearBufferReadyForTx = 0;
}

typedef enum eSystemMode
{
	eSysMode_StartUp = 0,
	eSysMode_Idle,
	eSysMode_Tx,
	eSysMode_FullScan,
	eSysMode_PartialScan,
	eSysMode_StaticScan
}T_System_Mode;

T_System_Mode xSysModeNow;
T_System_Mode xSysModeNext;
T_System_Mode xSysMode = eSysMode_Idle; //TODO: change to start up


#define STMD_ROTATION_INC_COARSE	   9000L
#define STMD_ROTATION_INC_MEDIUM	   4500L
#define STMD_ROTATION_INC_FINE		   1125L
#define STMD_ROTATION_360DEG		3600000L

typedef enum eSTMD_RetVals
{
	eSTMD_Err = 0,
	eSTMD_Ok
}T_STMD_RetVals;

typedef struct strSTMD_PinConfig
{
	GPIO_TypeDef *xSTMD_GPIO_Port_Pin_Enable;
	GPIO_TypeDef *xSTMD_GPIO_Port_Pin_Cfg1;
	GPIO_TypeDef * xSTMD_GPIO_Port_Pin_Step;
	GPIO_TypeDef *xSTMD_GPIO_Port_Pin_Dir;
	uint16_t u16STMD_GPIO_Pin_Enable;
	uint16_t u16STMD_GPIO_Pin_Cfg1;
	uint16_t u16STMD_GPIO_Pin_Step;
	uint16_t u16STMD_GPIO_Pin_Dir;
}T_STMD_PinConfig;

typedef struct strSTMD_Rotation
{
	int32_t i32STMD_CurrentAngle;
	int32_t i32STMD_MaxAngle;
	int32_t i32STMD_AngleIncrement;
}T_STMD_Rotation;

typedef struct strSTMD_Config
{
	T_STMD_PinConfig xSTMD_PinConfig;
	T_STMD_Rotation xSTMD_Rotation;
}T_STMD_Config;

typedef enum eSTMD_StepResolution
{
	eSTMD_Coarse = 0,
	eSTMD_Medium,
	eSMTD_Fine
}T_STMD_StepResolution;

typedef enum eSTMD_StepDirection
{
	eSTMD_CW = 0,
	eSTMD_CCW
}T_STMD_StepDirection;
/*
T_STMD_RetVals xSTMD_SetConfig(T_STMD_Config * pxSTMD_Config);

T_STMD_RetVals xSTMD_PerformHomingCycle(void);

T_STMD_RetVals xSTMD_PerformStep(T_STMD_StepDirection xStepDir, T_STMD_StepResolution xStepRes);
*/

T_STMD_Config xSTMD_Config;

void enableDriver(void)
{
	/* for readability */
	T_STMD_PinConfig * pxPinCfg = &xSTMD_Config.xSTMD_PinConfig;

	HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Enable,pxPinCfg->u16STMD_GPIO_Pin_Enable, GPIO_PIN_RESET);
}

void setStepResolution(T_STMD_StepResolution xStepRes)
{
	/* for readability */
	T_STMD_PinConfig * pxPinCfg = &xSTMD_Config.xSTMD_PinConfig;

	if(eSMTD_Fine == xStepRes)
	{
		HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Cfg1,pxPinCfg->u16STMD_GPIO_Pin_Cfg1,GPIO_PIN_RESET);
	}
	else if((eSTMD_Medium == xStepRes) || eSTMD_Coarse == xStepRes)
	{
		HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Cfg1,pxPinCfg->u16STMD_GPIO_Pin_Cfg1,GPIO_PIN_SET);
	}
}


T_STMD_RetVals xSTMD_SetConfig(T_STMD_Config * pxSTMD_Config)
{
	/* assign configuration */
	/* pin configuration */
	xSTMD_Config.xSTMD_PinConfig = pxSTMD_Config->xSTMD_PinConfig;
	/* rotation */
	/* verify settings */
	if((STMD_ROTATION_INC_FINE == pxSTMD_Config->xSTMD_Rotation.i32STMD_AngleIncrement) ||
			(STMD_ROTATION_INC_MEDIUM == pxSTMD_Config->xSTMD_Rotation.i32STMD_AngleIncrement) ||
			(STMD_ROTATION_INC_MEDIUM == pxSTMD_Config->xSTMD_Rotation.i32STMD_AngleIncrement))
	{
		xSTMD_Config.xSTMD_Rotation.i32STMD_AngleIncrement = pxSTMD_Config->xSTMD_Rotation.i32STMD_AngleIncrement;
	}
	else
	{
		return eSTMD_Err;
	}
	if((0 < pxSTMD_Config->xSTMD_Rotation.i32STMD_MaxAngle) &&
			(STMD_ROTATION_360DEG >= pxSTMD_Config->xSTMD_Rotation.i32STMD_MaxAngle))
	{
		xSTMD_Config.xSTMD_Rotation.i32STMD_MaxAngle = pxSTMD_Config->xSTMD_Rotation.i32STMD_MaxAngle;
	}
	else
	{
		return eSTMD_Err;
	}
	xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle = 0;

	enableDriver();

	/* return success */
	return eSTMD_Ok;
}

T_STMD_RetVals xSTMD_PerformHomingCycle(void)
{
	/* perform homing cycle*/

	xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle = 0;
	/* return success */
	return eSTMD_Ok;
}

T_STMD_RetVals xSTMD_PerformStep(T_STMD_StepDirection xStepDir, T_STMD_StepResolution xStepRes)
{
	/* for readability */
	T_STMD_Rotation * pxRotation = &xSTMD_Config.xSTMD_Rotation;
	T_STMD_PinConfig * pxPinCfg = &xSTMD_Config.xSTMD_PinConfig;


	/* set resolution */
	setStepResolution(xStepRes);

	/* check in which direction to turn */
	if(eSTMD_CW == xStepDir)
	{
		/* check if max angle is reached */
		if(pxRotation->i32STMD_MaxAngle >= (pxRotation->i32STMD_CurrentAngle + pxRotation->i32STMD_AngleIncrement))
		{
			/* set direction */
			HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Dir, pxPinCfg->u16STMD_GPIO_Pin_Dir, GPIO_PIN_RESET);
			/* step */
			HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Step, pxPinCfg->u16STMD_GPIO_Pin_Step, GPIO_PIN_SET);
			osDelay(1);
			HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Step, pxPinCfg->u16STMD_GPIO_Pin_Step, GPIO_PIN_RESET);
			/* sum up */
			pxRotation->i32STMD_CurrentAngle += pxRotation->i32STMD_AngleIncrement;
		}
		else
		{
			return eSTMD_Err;
		}
	}
	else if(eSTMD_CCW == xStepDir)
	{
		/* check if max angle is reached */
		if(0 <= (pxRotation->i32STMD_CurrentAngle - pxRotation->i32STMD_AngleIncrement))
		{
			/* set direction */
			HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Dir, pxPinCfg->u16STMD_GPIO_Pin_Dir, GPIO_PIN_SET);
			/* step */
			HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Step, pxPinCfg->u16STMD_GPIO_Pin_Step, GPIO_PIN_SET);
			osDelay(1);
			HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Step, pxPinCfg->u16STMD_GPIO_Pin_Step, GPIO_PIN_RESET);
			/* sum up */
			pxRotation->i32STMD_CurrentAngle -= pxRotation->i32STMD_AngleIncrement;
		}
		else
		{
			return eSTMD_Err;
		}
	}
	else
	{
		/* TODO: [STMD] Error handling: undefined step direction */
	}

	/* return success */
	return eSTMD_Ok;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	Init_Linear_Buffers();

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
  /* set system mode */
  xSysModeNow = eSysMode_Idle;
  xSysModeNext = eSysMode_Idle;
  /* configure stepper motor driver module */
  T_STMD_Config xConfig;
  xConfig.xSTMD_PinConfig.xSTMD_GPIO_Port_Pin_Enable = STMD_En_GPIO_Port;
  xConfig.xSTMD_PinConfig.u16STMD_GPIO_Pin_Enable = STMD_En_Pin;
  xConfig.xSTMD_PinConfig.xSTMD_GPIO_Port_Pin_Cfg1 = STMD_Cfg1_GPIO_Port;
  xConfig.xSTMD_PinConfig.u16STMD_GPIO_Pin_Cfg1 = STMD_Cfg1_Pin;
  xConfig.xSTMD_PinConfig.xSTMD_GPIO_Port_Pin_Dir = STMD_Dir_GPIO_Port;
  xConfig.xSTMD_PinConfig.u16STMD_GPIO_Pin_Dir = STMD_Dir_Pin;
  xConfig.xSTMD_PinConfig.xSTMD_GPIO_Port_Pin_Step = STMD_Step_GPIO_Port;
  xConfig.xSTMD_PinConfig.u16STMD_GPIO_Pin_Step = STMD_Step_Pin;
  xConfig.xSTMD_Rotation.i32STMD_AngleIncrement = STMD_ROTATION_INC_MEDIUM;
  xConfig.xSTMD_Rotation.i32STMD_CurrentAngle = 0;
  xConfig.xSTMD_Rotation.i32STMD_MaxAngle = STMD_ROTATION_360DEG;
  xSTMD_SetConfig(&xConfig);
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

  /* definition and creation of SensorsTask */
  osThreadDef(SensorsTask, SampleSensorsWork, osPriorityNormal, 0, 256);
  SensorsTaskHandle = osThreadCreate(osThread(SensorsTask), NULL);

  /* definition and creation of SMTD_Task */
  osThreadDef(SMTD_Task, SMTDWork, osPriorityNormal, 0, 256);
  SMTD_TaskHandle = osThreadCreate(osThread(SMTD_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of xQMainToSensor */
/* what about the sizeof here??? cd native code */
  osMessageQDef(xQMainToSensor, 3, T_MQ_Command);
  xQMainToSensorHandle = osMessageCreate(osMessageQ(xQMainToSensor), NULL);

  /* definition and creation of xQSensorToMain */
/* what about the sizeof here??? cd native code */
  osMessageQDef(xQSensorToMain, 3, T_MQ_Command);
  xQSensorToMainHandle = osMessageCreate(osMessageQ(xQSensorToMain), NULL);

  /* definition and creation of xQMainToSTMD */
/* what about the sizeof here??? cd native code */
  osMessageQDef(xQMainToSTMD, 3, T_MQ_Command);
  xQMainToSTMDHandle = osMessageCreate(osMessageQ(xQMainToSTMD), NULL);

  /* definition and creation of xQSTMDToMain */
/* what about the sizeof here??? cd native code */
  osMessageQDef(xQSTMDToMain, 3, T_MQ_Command);
  xQSTMDToMainHandle = osMessageCreate(osMessageQ(xQSTMDToMain), NULL);

  /* definition and creation of xQMainToCOM */
/* what about the sizeof here??? cd native code */
  osMessageQDef(xQMainToCOM, 3, T_MQ_Command);
  xQMainToCOMHandle = osMessageCreate(osMessageQ(xQMainToCOM), NULL);

  /* definition and creation of xQCOMToMain */
/* what about the sizeof here??? cd native code */
  osMessageQDef(xQCOMToMain, 3, T_MQ_Command);
  xQCOMToMainHandle = osMessageCreate(osMessageQ(xQCOMToMain), NULL);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STMD_Dir_Pin|STMD_Step_Pin|STMD_Cfg1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STMD_En_GPIO_Port, STMD_En_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SYS_LED_Pin */
  GPIO_InitStruct.Pin = SYS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STMD_Dir_Pin STMD_Step_Pin STMD_Cfg1_Pin */
  GPIO_InitStruct.Pin = STMD_Dir_Pin|STMD_Step_Pin|STMD_Cfg1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STMD_En_Pin */
  GPIO_InitStruct.Pin = STMD_En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STMD_En_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_LightBarrier_Pin */
  GPIO_InitStruct.Pin = IN_LightBarrier_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_LightBarrier_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

typedef enum eSensorSampling{
	eSensorSampling_Stopped = 0,
	eSensorSampling_Continuous,
	eSensorSampling_OneShot
}T_Sensor_Sampling_States;
T_Sensor_Sampling_States xSensorSamplingCurrentState = eSensorSampling_Stopped;
T_Sensor_Sampling_States xSensorSamplingNewState = eSensorSampling_Stopped;
T_Sensor_Sampling_States xSensorSamplingState = eSensorSampling_Stopped;


typedef enum{
	eMain_TxStopped = 0,
	eMain_TxContinuous,
	eMain_TxNSamples
}T_Main_Tx_States;
T_Main_Tx_States xMainTxState = eMain_TxStopped;

int32_t i32TxSampleCnt;

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

	received_data_size = 0;
	for(uint8_t u08FlushIdx=0; u08FlushIdx<100;u08FlushIdx++)
	{
		received_data[u08FlushIdx]=0;
	}
}

T_MQ_Command AnalyseRxCmd(void)
{
	char * cmd_perform_reduced_scan = "RSCAN %d";
	char * cmd_perform_complete_scan = "CSCAN";
	char * cmd_sample_distance_sensor = "DISTCONV";
	char * cmd_sample_distance_sensor_N = "DISTNCONV %d";
	char * cmd_stop_sample_distance_sensor = "STOPDISTCONV";
	T_MQ_Command xRetVal;
	xRetVal.xMQ_Cmd = eCMD_None;

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
		//xSysModeNext = eSysMode_FullScan;
		xRetVal.xMQ_Cmd = eCMD_FullScanMeasurement;
	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_sample_distance_sensor))
	{
		/* request to continuously sample the sensor */
		//xMainTxState = eMain_TxContinuous;
		//xSensorSamplingNewState = eSensorSampling_Continuous; /* ToDo: change to queue, bzw. als rückgabewert, damit task die queue befüllt */
		xRetVal.xMQ_Cmd = eCMD_SampleSensor;
		xRetVal.i32MQ_Cmd_ParamA = -1;

	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_stop_sample_distance_sensor))
	{
		/* request to stop continously sampling the sensor */
		//xMainTxState = eMain_TxStopped;
		//xSensorSamplingNewState = eSensorSampling_Stopped;
		xRetVal.xMQ_Cmd = eCMD_Abort;

	}
	else if(0 < sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_sample_distance_sensor_N, &i32Param))
	{
		if(0 < i32Param){
			if(UINT16_MAX >= i32Param)
			{
				//i32TxSampleCnt = i32Param;
				//xMainTxState = eMain_TxNSamples;
				//xSensorSamplingNewState = eSensorSampling_Continuous;
				xRetVal.xMQ_Cmd = eCMD_SampleSensor;
				xRetVal.i32MQ_Cmd_ParamA = i32Param;
			}
		}
	}
	return xRetVal;
}

#define DISTSENSE_VOUT_SAMPLING_CNT	1 /* ToDo: anpassen */
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

uint8_t au08DataOutput[1500];
//void Print_Sensor_Data(uint32_t u32Data)
void Print_Complete_Sensor_Data(T_LinearBuffer * pxLinBuff)
{
	static uint16_t su16MsgCnt = 0;
	volatile float fltADCVoltage;
	volatile uint16_t u16ADCVoltage;
	uint16_t u16StrIdx = 0;

	if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state)
	{

		for(uint16_t u16Idx=0; u16Idx < LINBUFFSIZE; u16Idx++)
		{
			fltADCVoltage = (float)pxLinBuff->au32Buffer[u16Idx];
			fltADCVoltage *= 3.3f;
			fltADCVoltage /= 4096;
			fltADCVoltage *= 100;
			u16ADCVoltage = (uint16_t)fltADCVoltage;
			u16StrIdx += sprintf((char *)&au08DataOutput[u16StrIdx], "%d %d %d %i\n", su16MsgCnt, pxLinBuff->au32Buffer[u16Idx],u16ADCVoltage, xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle);
		}


		su16MsgCnt++;
		CDC_Transmit_FS(&au08DataOutput[0],strlen(au08DataOutput));
	}
}

void Print_Sensor_Data(T_LinearBuffer * pxLinBuff)
{
	uint32_t u32Avg = 0;
	uint16_t u16StrIdx = 0;

	if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state)
	{
		/* calculate average */
		for(uint16_t u16Idx=0; u16Idx < LINBUFFSIZE; u16Idx++)
		{
			u32Avg += pxLinBuff->au32Buffer[u16Idx];
		}
		u32Avg /= LINBUFFSIZE;

		u16StrIdx += sprintf((char *)&au08DataOutput[u16StrIdx], "%i %d\n", xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle, u32Avg);

		CDC_Transmit_FS(&au08DataOutput[0],strlen(au08DataOutput));
	}
}

void Print_Sensor_Data_Value(uint32_t u32SensorVal)
{
	uint16_t u16StrIdx = 0;

	if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state)
	{

		u16StrIdx += sprintf((char *)&au08DataOutput[u16StrIdx], "%i %d\n", xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle, u32SensorVal);

		CDC_Transmit_FS(&au08DataOutput[0],strlen(au08DataOutput));
	}
}


typedef enum eLEDPattern
{
	eLED_Idle = 0,
	eLED_Running
}T_LED_Pattern;
#define LED_IDLE_ON		480
#define LED_IDLE_OFF	20
#define LED_RUNNING		100

inline void LED_Work(T_LED_Pattern xLEDPattern)
{
	static uint16_t su16LEDCnt = LED_IDLE_OFF;
	static T_LED_Pattern xLEDPatternLast = eLED_Idle;
	static GPIO_PinState sxLEDPinState = GPIO_PIN_RESET;

	/* check which led pattern was chosen */
	if(eLED_Running == xLEDPattern)
	{
		/* new state ? */
		if(xLEDPattern != xLEDPatternLast)
		{
			su16LEDCnt = LED_RUNNING;
			sxLEDPinState = GPIO_PIN_SET;
			/* take over state */
			xLEDPatternLast = xLEDPattern;
		}
		else
		{
			su16LEDCnt--;
			if(0 == su16LEDCnt)
			{
				if(GPIO_PIN_RESET == sxLEDPinState)
				{
					su16LEDCnt = LED_RUNNING;
					sxLEDPinState = GPIO_PIN_SET;
				}
				else
				{
					su16LEDCnt = LED_RUNNING;
					sxLEDPinState = GPIO_PIN_RESET;
				}
			}
		}
	}
	else if(eLED_Idle == xLEDPattern)
	{
		/* new state ? */
		if(xLEDPattern != xLEDPatternLast)
		{
			su16LEDCnt = LED_IDLE_ON;
			sxLEDPinState = GPIO_PIN_SET;
			/* take over state */
			xLEDPatternLast = xLEDPattern;
		}
		else
		{
			su16LEDCnt--;
			if(0 == su16LEDCnt)
			{
				if(GPIO_PIN_RESET == sxLEDPinState)
				{
					su16LEDCnt = LED_IDLE_ON;
					sxLEDPinState = GPIO_PIN_SET;
				}
				else
				{
					su16LEDCnt = LED_IDLE_OFF;
					sxLEDPinState = GPIO_PIN_RESET;
				}
			}
		}
	}

	/* write to led pin */
	HAL_GPIO_WritePin(SYS_LED_GPIO_Port,SYS_LED_Pin,sxLEDPinState);
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

	BaseType_t xMQRetVal;
	T_MQ_Command xMqCmd;
	T_MQ_Command xMqCOMCmd;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	uint8_t u08SampleSensorRQ = 0;

  /* Infinite loop */
	for(;;)
	{
		/* create fixed frequency task calling */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* check for incoming COM message */
		if(0 != xQCOMToMainHandle)
		{
			xMQRetVal = xQueueReceive(xQCOMToMainHandle, &xMqCOMCmd,0);
			if(pdPASS != xMQRetVal)
			{
				/* error or no message received */
				xMqCOMCmd.xMQ_Cmd = eCMD_None;
			}
		}

		/* check state */
		if(eSysMode_StartUp == xSysMode)
		{
			/* perform homing */
		}
		else if(eSysMode_FullScan == xSysMode)
		{
			static uint32_t u32InitialMeasurementStarted = 0;
			static T_STMD_StepDirection xCurrStepDir = eSTMD_CW;
			/* first check if an abort request was received */
			if(eCMD_Abort == xMqCOMCmd.xMQ_Cmd)
			{
				xSysMode = eSysMode_Idle;
			}
			else
			{
				/* prerequisite: homing done */
				if(0 == u32InitialMeasurementStarted)
				{
					/* start first measurement */
					/* request sampling */
					if(0 != xQMainToSensorHandle)
					{
						xMqCmd.xMQ_Cmd = eCMD_SampleSensor;
						xMQRetVal = xQueueSendToBack(xQMainToSensorHandle, &xMqCmd, 0);
						if(pdPASS == xMQRetVal)
						{
							/* sensor sampling request successful */
							u32InitialMeasurementStarted = 1;
						}
					}
				}
				else
				{
					/* check if measurement was done */
					if(0 != xQSensorToMainHandle)
					{
						xMQRetVal = xQueueReceive(xQSensorToMainHandle, &xMqCmd, 0);
						if(pdPASS == xMQRetVal)
						{
							/* evaluate message */
							if(eCMD_SensorValue == xMqCmd.xMQ_Cmd)
							{
								/* print sensor value */
								Print_Sensor_Data_Value((uint32_t)xMqCmd.i32MQ_Cmd_ParamA);
								/* perform next step */
								if(eSTMD_Ok == xSTMD_PerformStep(xCurrStepDir, eSTMD_Medium))
								{
									/* request next measurement */
									xMqCmd.xMQ_Cmd = eCMD_SampleSensor;
									xMQRetVal = xQueueSendToBack(xQMainToSensorHandle, &xMqCmd, 0);
									if(pdPASS != xMQRetVal)
									{
										/* sensor sampling request failed */
										xSysMode = eSysMode_Idle;	//TODO: appropriate error handling
									}
								}
								else
								{
									/* scan finished */
									u32InitialMeasurementStarted = 0;
									/* change scan direction for next scan */
									switch(xCurrStepDir)
									{
										case eSTMD_CW:
											xCurrStepDir = eSTMD_CCW;
											break;
										case eSTMD_CCW:
											xCurrStepDir = eSTMD_CW;
											break;
									}
									/* switch mode */
									xSysMode = eSysMode_Idle;
								}
							}
						}
					}
				}
			}
		}
		else if(eSysMode_StaticScan == xSysMode)
		{
			if(0 == u08SampleSensorRQ)
			{
				/* request sampling */
				if(0 != xQMainToSensorHandle)
				{
					xMqCmd.xMQ_Cmd = eCMD_SampleSensor;
					xMQRetVal = xQueueSendToBack(xQMainToSensorHandle, &xMqCmd, 0);
					if(pdPASS == xMQRetVal)
					{
						/* sensor sampling request successful */
						u08SampleSensorRQ = 1;
					}
				}
			}
			else
			{
				/* check response */
				if(0 != xQSensorToMainHandle)
				{
					/* message available ? */
					xMQRetVal = xQueueReceive(xQSensorToMainHandle, &xMqCmd, 0);
					if(pdPASS == xMQRetVal)
					{
						/* evaluate message */
						if(eCMD_SensorValue == xMqCmd.xMQ_Cmd)
						{
							/* print sensor value */
							Print_Sensor_Data_Value((uint32_t)xMqCmd.i32MQ_Cmd_ParamA);
							/* done successfully */
							xSysMode = eSysMode_Idle;
						}
					}
				}
			}
		}
		else if(eSysMode_Idle == xSysMode)
		{
			/* evaluate incoming com message */
			switch (xMqCOMCmd.xMQ_Cmd) {
				case eCMD_FullScanMeasurement:
					xSysMode = eSysMode_FullScan;
					break;
				case eCMD_SampleSensor:
					u08SampleSensorRQ = 0;
					xSysMode = eSysMode_StaticScan;
					break;
				case eCMD_StaticMeasurement:

					break;
				default:
					break;
			}
		}




		/* check incoming message queues */
		/* -> from sensor task */



		/* -> from stepper motor driver task */
		if(0 != xQSTMDToMainHandle)
		{

		}

		/* perform task depending on mode */
		if(1)
		{

		}
		else if(eSysMode_Tx == xSysModeNow)
		{
			/* decide if sensor data shall be transmitted via USB */
			if(eMain_TxContinuous == xMainTxState)
			{
				if(0 != pxLinearBufferReadyForTx)
				{
					/* transmit buffer content */
					Print_Complete_Sensor_Data(pxLinearBufferReadyForTx);
					/* flush buffer */
					LinearBufferFlush(pxLinearBufferReadyForTx);
					/* mark buffer as transmitted */
					pxLinearBufferReadyForTx = 0;
				}
			}
			else if(eMain_TxNSamples == xMainTxState)
			{
				if(0 < i32TxSampleCnt)
				{
					if(0 != pxLinearBufferReadyForTx)
					{
						/* count sample */
						i32TxSampleCnt--;
						/* transmit buffer content */
						Print_Complete_Sensor_Data(pxLinearBufferReadyForTx);
						/* flush buffer */
						LinearBufferFlush(pxLinearBufferReadyForTx);
						/* mark buffer as transmitted */
						pxLinearBufferReadyForTx = 0;
					}
				}
				else
				{
					xMainTxState = eMain_TxStopped;
					xSensorSamplingNewState = eSensorSampling_Stopped;
				}
			}
			/* take over next mode */
			xSysModeNow = xSysModeNext;
		}
		else if (eSysMode_FullScan == xSysModeNow)
		{
			static uint32_t u32HomingDone = 0;
			static uint32_t u32InitialMeasurementStarted = 0;
			T_STMD_RetVals xRetVal;
			static T_STMD_StepDirection xCurrStepDir = eSTMD_CW;
			if(0 == u32HomingDone)
			{
				/* perform homing cycle */
				xRetVal = xSTMD_PerformHomingCycle();
				u32HomingDone = 1;
			}
			if(0 == u32InitialMeasurementStarted)
			{
				/* start measurement */
				xSensorSamplingNewState = eSensorSampling_OneShot;
				u32InitialMeasurementStarted = 1;
			}
			else
			{
				/* tx measurement if buffer is full*/
				if(0 != pxLinearBufferReadyForTx)
				{
					/* transmit buffer content */
					Print_Sensor_Data(pxLinearBufferReadyForTx);
					/* flush buffer */
					LinearBufferFlush(pxLinearBufferReadyForTx);
					/* mark buffer as transmitted */
					pxLinearBufferReadyForTx = 0;
					/* perform step */
					if(eSTMD_Ok == xSTMD_PerformStep(xCurrStepDir, eSTMD_Medium))
					{
						/* start next measurement */
						xSensorSamplingNewState = eSensorSampling_OneShot;
						u32InitialMeasurementStarted = 1;
					}
					else
					{
						/* full scan is done => stop */
						xSysModeNext = eSysMode_Idle;
						u32InitialMeasurementStarted = 0;
						/* switch direction for next scan */
						if(eSTMD_CW == xCurrStepDir)
						{
							xCurrStepDir = eSTMD_CCW;
						}
						else if(eSTMD_CCW == xCurrStepDir)
						{
							xCurrStepDir = eSTMD_CW;
						}
					}
				}
			}
			/* take over next mode */
			xSysModeNow = xSysModeNext;
		}
		else if(eSysMode_Idle == xSysModeNow)
		{
			/* take over next mode */
			xSysModeNow = xSysModeNext;
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

	T_MQ_Command xMqCmd;

	/* Infinite loop */
	for(;;)
	{
		osDelay(100);	/* delay for 100 ms */
		/* clear buffer */
		for(u08Idx=0; u08Idx < sizeof(au08RxBuffer); u08Idx++)
		{
			au08RxBuffer[u08Idx] = 0;
		}
		/* check for incoming bytes on USB */
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

		//TODO: [main] es muss noch der USB buffer geflusht werden

		if(1 == xUartRxCmd.u08CommandReceived)
		{
			xMqCmd = AnalyseRxCmd();
			FlushRxCmd();

			/* check if a valid command was received */
			if(eCMD_None != xMqCmd.xMQ_Cmd)
			{
				/* inform main task about new command */
				if(0 != xQCOMToMainHandle)
				{
					xQueueSendToBack(xQCOMToMainHandle,&xMqCmd,0);
				}
			}

		}
	}
  /* USER CODE END COMTaskWork */
}

/* SampleSensorsWork function */
void SampleSensorsWork(void const * argument)
{
  /* USER CODE BEGIN SampleSensorsWork */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1; // every 1 ms
	uint8_t u08RetVal;

	volatile uint32_t u32ADCVal;
	volatile uint32_t u32DistSensorVal;
	volatile uint8_t u08MeasurementCnt;

	BaseType_t xQueueRetVal;
	T_MQ_Command xMainTaskCmd;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	/* Infinite loop */
	for(;;)
	{
		/* create fixed frequency task calling */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		/* check for an incoming message */
		/* verify that queue exists */
		if(0 != xQMainToSensorHandle)
		{
			xQueueRetVal = xQueueReceive(xQMainToSensorHandle, &xMainTaskCmd, 0);
			if(pdTRUE == xQueueRetVal)
			{
				/* evaluate the command from the main task */
				if(eCMD_SampleSensor == xMainTaskCmd.xMQ_Cmd)
				{
					/* delete average and start sampling */
					u32DistSensorVal = 0;
					u08MeasurementCnt = DIST_SENSOR_SAMPLING_CNT;
					xSensorSamplingState = eSensorSampling_OneShot;
				}
				else if(eCMD_Abort == xMainTaskCmd.xMQ_Cmd)
				{
					/* stop ongoing measurement */
					xSensorSamplingCurrentState = eSensorSampling_Stopped;
					xSensorSamplingNewState = eSensorSampling_Stopped;
					/* confirm to the main task */
					if(0 != xQSensorToMainHandle)
					{
						xMainTaskCmd.xMQ_Cmd = eCMD_Accepted;
						xQueueRetVal = xQueueSendToBack(xQSensorToMainHandle,&xMainTaskCmd,0);
					}
				}
			}
		}

		/* work */
		if(eSensorSampling_OneShot == xSensorSamplingState)
		{
			LED_Work(eLED_Running);

			if(0 != u08MeasurementCnt)
			{
				/* sample distance sensor and sum up result */
				u32DistSensorVal += Distance_ConvertSensorOutput();
				u08MeasurementCnt--;
			}
			else
			{
				/* finished sampling */
				/* calculate average */
				u32DistSensorVal /= DIST_SENSOR_SAMPLING_CNT;
				/* prepare message to Main task */
				xMainTaskCmd.xMQ_Cmd = eCMD_SensorValue;
				xMainTaskCmd.i32MQ_Cmd_ParamA = (int32_t)u32DistSensorVal;
				/* send via queue */
				if(0 != xQSensorToMainHandle)
				{
					xQueueSendToBack(xQSensorToMainHandle,&xMainTaskCmd,0);
				}
				/* stop sampling */
				xSensorSamplingState = eSensorSampling_Stopped;
			}
		}
		else
		{
			LED_Work(eLED_Idle);
		}

		if(1)
		{

		}
		else if(eSensorSampling_Continuous == xSensorSamplingCurrentState)
		{
			LED_Work(eLED_Running);
			/* sample distance sensor */
			u32ADCVal = Distance_ConvertSensorOutput();
			/* put value onto buffer */
			u08RetVal = u08LinearBufferPush(pxLinearBufferActive, u32ADCVal);
			/* check if buffer is full */
			if(0 == u08RetVal)
			{
				/* buffer is full */
				/* mark this buffer as ready for tx */
				pxLinearBufferReadyForTx = pxLinearBufferActive;
				/* switch to other buffer */
				if(&xLinearBuffer_1 == pxLinearBufferActive)
				{
					pxLinearBufferActive = &xLinearBuffer_2;
				}
				else if(&xLinearBuffer_2 == pxLinearBufferActive)
				{
					pxLinearBufferActive = &xLinearBuffer_1;
				}
				/* take over new state */
				xSensorSamplingCurrentState = xSensorSamplingNewState;
			}
		}
		else if(eSensorSampling_OneShot == xSensorSamplingCurrentState)
		{
			LED_Work(eLED_Running);
			/* sample distance sensor */
			u32ADCVal = Distance_ConvertSensorOutput();
			/* put value onto buffer */
			u08RetVal = u08LinearBufferPush(pxLinearBufferActive, u32ADCVal);
			/* check if buffer is full */
			if(0 == u08RetVal)
			{
				/* buffer is full */
				/* mark this buffer as ready for tx */
				pxLinearBufferReadyForTx = pxLinearBufferActive;
				/* take over new state */
				xSensorSamplingNewState = eSensorSampling_Stopped;
				xSensorSamplingCurrentState = xSensorSamplingNewState;
			}
		}
		else if(eSensorSampling_Stopped == xSensorSamplingCurrentState)
		{
			LED_Work(eLED_Idle);
			/* check if sensor sampling shall be enabled */
			if(eSensorSampling_Continuous == xSensorSamplingNewState)
			{
				/* init buffers and pointers */
				Init_Linear_Buffers();
				/* and take over new state */
				xSensorSamplingCurrentState = xSensorSamplingNewState;
			}
			else if(eSensorSampling_OneShot == xSensorSamplingNewState)
			{
				/* init buffers and pointers */
				Init_Linear_Buffers();
				/* and take over new state */
				xSensorSamplingCurrentState = xSensorSamplingNewState;
			}
		}
	}
  /* USER CODE END SampleSensorsWork */
}

/* SMTDWork function */
void SMTDWork(void const * argument)
{
  /* USER CODE BEGIN SMTDWork */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END SMTDWork */
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
