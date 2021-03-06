
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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

osThreadId Main_TaskHandle;
osThreadId COM_TaskHandle;
osThreadId SMTD_TaskHandle;
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
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void MainTaskWork(void const * argument);
void COMTaskWork(void const * argument);
void SMTDWork(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t) 100)     /* Size of array containing ADC converted values */
/* Variable containing ADC conversions results */
__IO uint16_t au16ADCValues[ADCCONVERTEDVALUES_BUFFER_SIZE];


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
	eCMD_ContinousScanMeasurment,
	eCMD_SampleSensor,
	eCMD_SensorValueReady,
	eCMD_PerformStep,
	eCMD_MoveToAngle,
	eCMD_PrintSTMDConfig,
	eCMD_SetAngleResolution,
	eCMD_SetStepResolution,
	eCMD_LimitSwitchHit,
	eCMD_Abort,
	eCMD_StartTestMode
}T_MQ_CommandList;

/* structure to hold commands and their parameters which will be passed between tasks */
typedef struct strMQ_Command
{
	T_MQ_CommandList xMQ_Cmd;
	int32_t i32MQ_Cmd_ParamA;
	int32_t i32MQ_Cmd_ParamB;
	int32_t i32MQ_Cmd_ParamC;
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
	eSysMode_ContinousScan,
	eSysMode_StaticScan,
	eSysMode_Homing,
	eSysMode_MoveToAngle,
	eSysMode_TestMode
}T_System_Mode;

T_System_Mode xSysMode = eSysMode_Idle; //TODO: change to start up


#define STMD_ROTATION_INC_COARSE	  45000L
#define STMD_ROTATION_INC_FINE		  22500L
#define STMD_ROTATION_360DEG		3600000L

#define STMD_STEP_INC_COARSE	4500L
#define STMD_STEP_INC_FINE		1125L

typedef enum eSTMD_RetVals
{
	eSTMD_Err = 0,
	eSTMD_Ok
}T_STMD_RetVals;

typedef enum eSTMD_StepResolution
{
	eSTMD_StepRes_Coarse = 0,
	eSTMD_StepRes_Fine
}T_STMD_StepResolution;

typedef enum eSTMD_StepDirection
{
	eSTMD_CW = 0,
	eSTMD_CCW
}T_STMD_StepDirection;

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
	int32_t i32STMD_StepIncrement;
	T_STMD_StepResolution xStepResolution;
	T_STMD_StepDirection xStepDirection;
}T_STMD_Rotation;

typedef struct strSTMD_Config
{
	T_STMD_PinConfig xSTMD_PinConfig;
	T_STMD_Rotation xSTMD_Rotation;
}T_STMD_Config;

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

	if(eSTMD_StepRes_Fine == xStepRes)
	{
		HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Cfg1,pxPinCfg->u16STMD_GPIO_Pin_Cfg1,GPIO_PIN_RESET);
		xSTMD_Config.xSTMD_Rotation.i32STMD_StepIncrement = STMD_STEP_INC_FINE;
	}
	else if(eSTMD_StepRes_Coarse == xStepRes)
	{
		HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Cfg1,pxPinCfg->u16STMD_GPIO_Pin_Cfg1,GPIO_PIN_SET);
		xSTMD_Config.xSTMD_Rotation.i32STMD_StepIncrement = STMD_STEP_INC_COARSE;
	}
}


T_STMD_RetVals xSTMD_SetConfig(T_STMD_Config * pxSTMD_Config)
{
	/* assign configuration */
	/* pin configuration */
	xSTMD_Config.xSTMD_PinConfig = pxSTMD_Config->xSTMD_PinConfig;
	/* rotation */
	/* verify settings */
	//if((STMD_ROTATION_INC_FINE == pxSTMD_Config->xSTMD_Rotation.i32STMD_AngleIncrement) ||
	//		(STMD_ROTATION_INC_COARSE == pxSTMD_Config->xSTMD_Rotation.i32STMD_AngleIncrement))
	if(eSTMD_StepRes_Coarse == pxSTMD_Config->xSTMD_Rotation.xStepResolution)
	{
		xSTMD_Config.xSTMD_Rotation.i32STMD_AngleIncrement = STMD_ROTATION_INC_COARSE;
		xSTMD_Config.xSTMD_Rotation.i32STMD_StepIncrement = STMD_STEP_INC_COARSE;
	}
	else if(eSTMD_StepRes_Fine == pxSTMD_Config->xSTMD_Rotation.xStepResolution)
	{
		xSTMD_Config.xSTMD_Rotation.i32STMD_AngleIncrement = STMD_ROTATION_INC_FINE;
		xSTMD_Config.xSTMD_Rotation.i32STMD_StepIncrement = STMD_STEP_INC_FINE;
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

void xSTMD_PrintConfig(char * pConfigString)
{
	int32_t i32Idx=0;

	/* Angle per Step */
	i32Idx = sprintf(pConfigString, "Angle inc. per step: %li\n",xSTMD_Config.xSTMD_Rotation.i32STMD_StepIncrement);
	pConfigString+=i32Idx;
	/* Angle increment */
	i32Idx = sprintf(pConfigString,"Angle inc.: %li\n",xSTMD_Config.xSTMD_Rotation.i32STMD_AngleIncrement);
	pConfigString+=i32Idx;
	/* Current position */
	i32Idx = sprintf(pConfigString,"Current angle: %li\n",xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle);
	pConfigString+=i32Idx;
	/* Max Angle */
	i32Idx = sprintf(pConfigString,"Max. angle: %li\n",xSTMD_Config.xSTMD_Rotation.i32STMD_MaxAngle);
}

/* Function called at the end of a DMA ADC transfer */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	/* DMA transfer completed => inform main task */
	T_MQ_Command xMainTaskCmd;
	BaseType_t bTaskWoken;

	if(0 != xQSensorToMainHandle)
	{
		xMainTaskCmd.xMQ_Cmd = eCMD_SensorValueReady;

		xQueueSendToBackFromISR(xQSensorToMainHandle, &xMainTaskCmd, &bTaskWoken);
	}
}


void SensorSamplingInit(uint16_t * pu16ADCValues)
{
	/* clear buffer */
	for(uint32_t u32Idx=0; u32Idx < ADCCONVERTEDVALUES_BUFFER_SIZE; u32Idx++)
	{
		pu16ADCValues[u32Idx] = 0;
	}
	/* start adc */
}

void SensorSamplingStart(void)
{
	/* (re)start ADC */
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)au16ADCValues, ADCCONVERTEDVALUES_BUFFER_SIZE) != HAL_OK)
	{
		/* Start Error */
		//  Error_Handler();
	}
}

uint32_t u32SensorSamplingGetAverage(uint16_t * pu16ADCValues)
{
	uint32_t u32ADCAverage = 0;

	for(uint32_t u32Idx=0; u32Idx < ADCCONVERTEDVALUES_BUFFER_SIZE; u32Idx++)
	{
		u32ADCAverage += pu16ADCValues[u32Idx];
		pu16ADCValues[u32Idx] = 0;
	}

	return u32ADCAverage / ADCCONVERTEDVALUES_BUFFER_SIZE;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* start TIM3, which will be triggering the ADC conversions */
  HAL_TIM_Base_Start_IT(&htim3);
  /* prepare ADC */
  SensorSamplingInit(&au16ADCValues[0]);
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
  xConfig.xSTMD_Rotation.i32STMD_CurrentAngle = 0;
  xConfig.xSTMD_Rotation.i32STMD_MaxAngle = STMD_ROTATION_360DEG;
  xConfig.xSTMD_Rotation.xStepDirection = eSTMD_CW;
  xConfig.xSTMD_Rotation.xStepResolution = eSTMD_StepRes_Fine;
  xSTMD_SetConfig(&xConfig);
  /* set resolution */
  setStepResolution(xConfig.xSTMD_Rotation.xStepResolution);
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

  /* definition and creation of SMTD_Task */
  osThreadDef(SMTD_Task, SMTDWork, osPriorityNormal, 0, 256);
  SMTD_TaskHandle = osThreadCreate(osThread(SMTD_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
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
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
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

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 36000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
	char * cmd_perform_partial_scan = "PSCAN %d %d %d";
	char * cmd_perform_continuous_scan = "CSCAN %d";
	char * cmd_perform_full_scan = "FSCAN";
	char * cmd_sample_distance_sensor = "DISTCONV";
	char * cmd_sample_distance_sensor_N = "DISTNCONV %d";
	char * cmd_stop_sample_distance_sensor = "STOPDISTCONV";
	char * cmd_print_stmd_config = "CSTMD";
	char * cmd_set_angle_resolution = "ANGLERES %d";
	char * cmd_set_step_resolution = "STEPRES %d";
	char * cmd_goto_position = "POS %d";
	char * cmd_perform_homing = "HOME";
	char * cmd_perform_testmode = "TEST";
	char * cmd_abort = "ABORT";

	T_MQ_Command xRetVal;
	xRetVal.xMQ_Cmd = eCMD_None;

	int32_t i32Param = 0;
	int32_t i32ParamB = 0;
	int32_t i32ParamC = 0;

	/* analyse received command	*/
	if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_abort))
		{
			xRetVal.xMQ_Cmd = eCMD_Abort;
		}
	else if(0 < sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_perform_partial_scan,&i32Param,&i32ParamB, &i32ParamC))
	{
		/* perform partial scan from angle i32ParamB to i32ParamC and take i32Param samples at each angle */
		xRetVal.xMQ_Cmd = eCMD_PartialScanMeasurement;
		xRetVal.i32MQ_Cmd_ParamA = i32Param;
		xRetVal.i32MQ_Cmd_ParamB = i32ParamB;
		xRetVal.i32MQ_Cmd_ParamC = i32ParamC;
	}
	else if(0 < sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_set_angle_resolution,&i32Param))
	{
		xRetVal.xMQ_Cmd = eCMD_SetAngleResolution;
		xRetVal.i32MQ_Cmd_ParamA = i32Param;
	}
	else if(0 < sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_set_step_resolution,&i32Param))
	{
		xRetVal.xMQ_Cmd = eCMD_SetStepResolution;
		xRetVal.i32MQ_Cmd_ParamA = i32Param;
	}
	else if(0 < sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_goto_position,&i32Param))
	{
		xRetVal.xMQ_Cmd = eCMD_MoveToAngle;
		xRetVal.i32MQ_Cmd_ParamA = i32Param;
	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_perform_homing))
	{
		xRetVal.xMQ_Cmd = eCMD_PerformHoming;
	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_perform_full_scan))
	{
		/* perform a full resolution scan */
		xRetVal.xMQ_Cmd = eCMD_FullScanMeasurement;
	}
	else if(0 < sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_perform_continuous_scan,&i32Param))
	{
		xRetVal.i32MQ_Cmd_ParamA = i32Param;
		xRetVal.xMQ_Cmd = eCMD_ContinousScanMeasurment;
	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_sample_distance_sensor))
	{
		/* request to continuously sample the sensor */
		xRetVal.xMQ_Cmd = eCMD_SampleSensor;
		xRetVal.i32MQ_Cmd_ParamA = -1;

	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_stop_sample_distance_sensor))
	{
		/* request to stop continously sampling the sensor */
		xRetVal.xMQ_Cmd = eCMD_Abort;

	}
	else if(0 < sscanf((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_sample_distance_sensor_N, &i32Param))
	{
		if(0 < i32Param){
			if(UINT16_MAX >= i32Param)
			{
				xRetVal.xMQ_Cmd = eCMD_SampleSensor;
				xRetVal.i32MQ_Cmd_ParamA = i32Param;
			}
		}
	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_print_stmd_config))
	{
		xRetVal.xMQ_Cmd = eCMD_PrintSTMDConfig;
	}
	else if(0 == strcmp((const char *)&xUartRxCmd.au08CommandBuffer[0], cmd_perform_testmode))
	{
		xRetVal.xMQ_Cmd = eCMD_StartTestMode;
	}
	return xRetVal;
}

uint8_t au08DataOutput[1500];

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

		u16StrIdx += sprintf((char *)&au08DataOutput[u16StrIdx], "%li %ld\n", xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle, u32Avg);

		CDC_Transmit_FS(&au08DataOutput[0],strlen(au08DataOutput));
	}
}

void Print_Sensor_Data_Value(uint32_t u32SensorVal)
{
	uint16_t u16StrIdx = 0;

	if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state)
	{

		//u16StrIdx += sprintf((char *)&au08DataOutput[u16StrIdx], "%li %ld\n", xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle, u32SensorVal);
		u16StrIdx += sprintf((char *)&au08DataOutput[u16StrIdx], "%li %li\n", xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle, i32GetDistanceFromADCVal((uint16_t)u32SensorVal));

		CDC_Transmit_FS(&au08DataOutput[0],strlen(au08DataOutput));
	}
}

void Print_Test_Data_Value(uint16_t u16SensorVal)
{
	uint16_t u16StrIdx = 0;

	if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state)
	{

		//u16StrIdx += sprintf((char *)&au08DataOutput[u16StrIdx], "%li %ld\n", xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle, u16SensorVal);
		u16StrIdx += sprintf((char *)&au08DataOutput[u16StrIdx], "%d %li\n", u16SensorVal, i32GetDistanceFromADCVal(u16SensorVal));

		CDC_Transmit_FS(&au08DataOutput[0],strlen(au08DataOutput));
	}
}
void Print_Config(void)
{
	if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state)
	{
		/* get config */
		xSTMD_PrintConfig((char *)&au08DataOutput[0]);
		/* send */
		CDC_Transmit_FS(&au08DataOutput[0],strlen(au08DataOutput));
	}
}

typedef enum eWorkStates
{
	eWork_Abort = 0,
	eWork_Started,
	eWork_ReqMove,
	eWork_WaitMove,
	eWork_ReqSampling,
	eWork_WaitSampling,
	eWork_PrintData,
	eWork_Done,
	eWork_Idle,
	eWork_Moving,
	eWork_LimitSwitchHit
}T_WorkStates;

typedef enum eMove_LimitSwitch
{
	eMove_IgnoreLimitSwitch = 0,
	eMove_CheckLimitSwitch
}T_Move_LimitSwitch;

T_WorkStates xScanWork(T_WorkStates xWorkState, T_STMD_StepDirection xStepDir, int32_t i32StartAngle, int32_t i32StopAngle, uint16_t u16SampleCount)
{
	static int32_t si32NextAngle = 0;
	static uint16_t su16SCount = 0;
	static uint32_t u32SensorVal = 0;
	T_MQ_Command xMqCommand;

	switch (xWorkState) {
		case eWork_Started:
			/* verify given parameters */
			/* take over parameters */
			su16SCount = u16SampleCount;
			/* move to start angle */
			if(xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle != i32StartAngle)
			{
				/* request to move to angle */
				si32NextAngle = i32StartAngle;
				xWorkState = eWork_ReqMove;
			}
			else
			{
				/* already there */
				si32NextAngle = i32StartAngle;
				/* => directly request sensor sampling */
				xWorkState = eWork_ReqSampling;
			}
			/* determine the direction of turning */
			if(i32StartAngle <= i32StopAngle)
			{
				xSTMD_Config.xSTMD_Rotation.xStepDirection = eSTMD_CW;
			}
			else if(i32StartAngle > i32StopAngle)
			{
				xSTMD_Config.xSTMD_Rotation.xStepDirection = eSTMD_CCW;
			}
			break;
		case eWork_ReqMove:
			/* request to move to angle */
			if(0 != xQMainToSTMDHandle)
			{
				/* prepare command */
				xMqCommand.xMQ_Cmd = eCMD_MoveToAngle;
				xMqCommand.i32MQ_Cmd_ParamA = si32NextAngle;
				xMqCommand.i32MQ_Cmd_ParamB = eMove_IgnoreLimitSwitch;
				/* send command */
				if(pdPASS == xQueueSendToBack(xQMainToSTMDHandle,&xMqCommand,0))
				{
					/* wait for move to finish */
					xWorkState = eWork_WaitMove;
				}
				else
				{
					/* ERR: Item not put on queue */
					xWorkState = eWork_Abort;
				}
			}
			else
			{
				/* ERR: Queue not available */
				xWorkState = eWork_Abort;
			}
			break;
		case eWork_WaitMove:
			/* check for response of stepper motor driver task */
			/* Queue available ? */
			if(0 != xQSTMDToMainHandle)
			{
				if(pdPASS == xQueueReceive(xQSTMDToMainHandle,&xMqCommand, 0))
				{
					/* evaluate response */
					if(eCMD_Accepted == xMqCommand.xMQ_Cmd)
					{
						/* angle reached -> request sampling */
						xWorkState = eWork_ReqSampling;
					}
					else
					{
						/* ERR: Angle could not be reached */
						xWorkState = eWork_Abort;
					}
				}
				else
				{
					/* no message on the queue -> keep state */
					xWorkState = eWork_WaitMove;
					/*ToDo: error handling, if no message arrived after x seconds */
				}
			}
			else
			{
				/* ERR: Queue not available */
				xWorkState = eWork_Abort;
			}
			break;
		case eWork_ReqSampling:
			/* start sensor sampling */
			SensorSamplingStart();
			/* wait for sampling to finish */
			xWorkState = eWork_WaitSampling;
			break;
		case eWork_WaitSampling:
			/* check if sampling is done */
			if(0 != xQSensorToMainHandle)
			{
				if(pdPASS == xQueueReceive(xQSensorToMainHandle,&xMqCommand,0))
				{
					/* evaluate response */
					if(eCMD_SensorValueReady == xMqCommand.xMQ_Cmd)
					{
						/* get value */
						u32SensorVal = u32SensorSamplingGetAverage(&au16ADCValues[0]);
						xWorkState = eWork_PrintData;
					}
					else
					{
						/* sampling failed */
						xWorkState = eWork_Abort;
					}
				}
				else
				{
					/* no message on the queue -> keep state */
					xWorkState = eWork_WaitSampling;
					/*ToDo: error handling, if no message after x seconds */
				}
			}
			else
			{
				/* ERR: Queue not available */
				xWorkState = eWork_Abort;
			}
			break;
		case eWork_PrintData:
			/* print data */
			Print_Sensor_Data_Value(u32SensorVal);
			/* count sample */
			su16SCount--;
			if(0 != su16SCount)
			{
				/* take another sample at this position */
				xWorkState = eWork_ReqSampling;
			}
			else
			{
				/* sampling done for the current angle */
				su16SCount = u16SampleCount;
				/* check if end position was reached */
				if(eSTMD_CW == xSTMD_Config.xSTMD_Rotation.xStepDirection)
				{
					if(xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle < i32StopAngle)
					{
						/* set next angle */
						si32NextAngle = xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle + xSTMD_Config.xSTMD_Rotation.i32STMD_AngleIncrement;
						/* and request move */
						xWorkState = eWork_ReqMove;
					}
					else
					{
						/* stop */
						xWorkState = eWork_Done;
					}
				}
				else
				{
					if(xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle > i32StopAngle)
					{
						/* set next angle */
						si32NextAngle = xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle - xSTMD_Config.xSTMD_Rotation.i32STMD_AngleIncrement;
						/* and request move */
						xWorkState = eWork_ReqMove;
					}
					else
					{
						/* stop */
						xWorkState = eWork_Done;
					}
				}
			}
			break;
		case eWork_Done:
			/* switch back to idle */
			xWorkState = eWork_Started;
			break;
		case eWork_Abort:
		default:
			break;
	}
	return xWorkState;
}

T_WorkStates xMoveWork(T_WorkStates xWorkState, int32_t i32TargetAngle, T_Move_LimitSwitch xLimitSwitch)
{
	/* for readability */
	T_STMD_Rotation * pxRotation = &xSTMD_Config.xSTMD_Rotation;


	T_MQ_Command xMqCommand;

	switch (xWorkState) {
		case eWork_Started:
			/* check if move is needed */
			if(i32TargetAngle == pxRotation->i32STMD_CurrentAngle)
			{
				/* done */
				xWorkState = eWork_Done;
			}
			else
			{
				/* request move */
				xWorkState = eWork_ReqMove;
			}
			break;
		case eWork_ReqMove:
			/* request to move to angle */
			if(0 != xQMainToSTMDHandle)
			{
				/* prepare command */
				xMqCommand.xMQ_Cmd = eCMD_MoveToAngle;
				xMqCommand.i32MQ_Cmd_ParamA = i32TargetAngle;
				xMqCommand.i32MQ_Cmd_ParamB = xLimitSwitch;
				/* send command */
				if(pdPASS == xQueueSendToBack(xQMainToSTMDHandle,&xMqCommand,0))
				{
					/* wait for move to finish */
					xWorkState = eWork_WaitMove;
				}
				else
				{
					/* ERR: Item not put on queue */
					xWorkState = eWork_Abort;
				}
			}
			else
			{
				/* ERR: Queue not available */
				xWorkState = eWork_Abort;
			}
			break;
		case eWork_WaitMove:
			/* check for response of stepper motor driver task */
			/* Queue available ? */
			if(0 != xQSTMDToMainHandle)
			{
				if(pdPASS == xQueueReceive(xQSTMDToMainHandle,&xMqCommand, 0))
				{
					/* evaluate response */
					if(eCMD_Accepted == xMqCommand.xMQ_Cmd)
					{
						/* angle reached -> done */
						xWorkState = eWork_Done;
					}
					else if(eCMD_LimitSwitchHit == xMqCommand.xMQ_Cmd)
					{
						/* ran into limit switch */
						xWorkState = eWork_LimitSwitchHit;
					}
					else
					{
						/* ERR: Angle could not be reached */
						xWorkState = eWork_Abort;
					}
				}
				else
				{
					/* no message on the queue -> keep state */
					xWorkState = eWork_WaitMove;
					/*ToDo: error handling, if no message arrived after x seconds */
				}
			}
			else
			{
				/* ERR: Queue not available */
				xWorkState = eWork_Abort;
			}
			break;
		default:
			//xWorkState = eWork_Started;
			break;
	}

	return xWorkState;
}


T_WorkStates xSTMD_PerformStepping(T_STMD_StepDirection xStepDir, int32_t i32StepCnt, T_Move_LimitSwitch xLimitSwitch)
{
	T_WorkStates xRetVal = eWork_Done;
	/* for readability */
	T_STMD_Rotation * pxRotation = &xSTMD_Config.xSTMD_Rotation;
	T_STMD_PinConfig * pxPinCfg = &xSTMD_Config.xSTMD_PinConfig;
	GPIO_PinState xLimitSwitchState1 = GPIO_PIN_SET;
	GPIO_PinState xLimitSwitchState2 = GPIO_PIN_SET;

	#define RAMPUP_LENGTH	6
	uint8_t au08RampUp[RAMPUP_LENGTH] ={20, 17, 12, 7, 6, 5};
	uint8_t u08RampUpIdx=RAMPUP_LENGTH;

	/* 1.) choose in which direction to step */
	if(eSTMD_CW == pxRotation->xStepDirection)
	{
		/* set direction */
		HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Dir, pxPinCfg->u16STMD_GPIO_Pin_Dir, GPIO_PIN_RESET);
	}
	else
	{
		/* set direction */
		HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Dir, pxPinCfg->u16STMD_GPIO_Pin_Dir, GPIO_PIN_SET);
	}

	/* 2.) perform stepping */
	while(0 < i32StepCnt)
	{
		/* step */
		HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Step, pxPinCfg->u16STMD_GPIO_Pin_Step, GPIO_PIN_SET);
		osDelay(au08RampUp[RAMPUP_LENGTH - u08RampUpIdx]);
		HAL_GPIO_WritePin(pxPinCfg->xSTMD_GPIO_Port_Pin_Step, pxPinCfg->u16STMD_GPIO_Pin_Step, GPIO_PIN_RESET);
		xLimitSwitchState1 = HAL_GPIO_ReadPin(IN_LightBarrier_GPIO_Port,IN_LightBarrier_Pin);
		osDelay(au08RampUp[RAMPUP_LENGTH - u08RampUpIdx]);
		/* count step */
		i32StepCnt--;
		/* adjust current angle */
		pxRotation->i32STMD_CurrentAngle += pxRotation->i32STMD_StepIncrement;
		/* speed control */
		if(u08RampUpIdx)
		{
			u08RampUpIdx--;
		}
		else
		{
			u08RampUpIdx = 1;
		}
		if(eMove_CheckLimitSwitch == xLimitSwitch)
		{
			/* check if limit switch was hit */
			xLimitSwitchState2 = HAL_GPIO_ReadPin(IN_LightBarrier_GPIO_Port,IN_LightBarrier_Pin);
			if(xLimitSwitchState2 == xLimitSwitchState1)
			{
				if(GPIO_PIN_SET == xLimitSwitchState2)
				{
					/* limit switch hit */
					xRetVal = eWork_LimitSwitchHit;
					break;
				}
			}
		}
	}
	return xRetVal;
}

/* USER CODE END 4 */

/* MainTaskWork function */
void MainTaskWork(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10; // every 25 ms

	BaseType_t xMQRetVal;
	T_MQ_Command xMqCmd;
	T_MQ_Command xMqCOMCmd;

	static int32_t si32StartAngle, si32StopAngle;
	static uint16_t su16SampleCount;
	volatile T_WorkStates xWorkTaskResponse =  eWork_Started;

//	T_WorkStates xMoveWorkState = eWork_Started;

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
			/* preload message */
			xMqCOMCmd.xMQ_Cmd = eCMD_None;
			/* check for new message */
			xQueueReceive(xQCOMToMainHandle, &xMqCOMCmd,0);
		}

		/* check state */
		if(eSysMode_StartUp == xSysMode)
		{
			/* perform homing */
		}
		else if(eSysMode_FullScan == xSysMode)
		{
		}
		else if(eSysMode_PartialScan == xSysMode)
		{
			/* first check if an abort request was received */
			if(eCMD_Abort == xMqCOMCmd.xMQ_Cmd)
			{
				xSysMode = eSysMode_Idle;
			}
			else
			{
				/* perform scanning */
				xWorkTaskResponse = xScanWork(xWorkTaskResponse, eSTMD_CW,si32StartAngle,si32StopAngle,su16SampleCount);
				if((eWork_Abort == xWorkTaskResponse) || (eWork_Done == xWorkTaskResponse))
				{
					/* error or done -> switch to idle mode */
					xSysMode = eSysMode_Idle;
				}
				else
				{
					/* keep state */
					xSysMode = eSysMode_PartialScan;
				}
			}
		}
		else if(eSysMode_ContinousScan == xSysMode)
		{
			/* first check if an abort request was received */
			if(eCMD_Abort == xMqCOMCmd.xMQ_Cmd)
			{
				xSysMode = eSysMode_Idle;
			}
			else
			{
				/* perform scanning */
				xWorkTaskResponse = xScanWork(xWorkTaskResponse, eSTMD_CW,si32StartAngle,si32StopAngle,su16SampleCount);
				if(eWork_Abort == xWorkTaskResponse)
				{
					/* error during scan => abort */
					xSysMode = eSysMode_Idle;
				}
				else if(eWork_Done == xWorkTaskResponse)
				{
					/* change turn direction */
					if(eSTMD_CW == xSTMD_Config.xSTMD_Rotation.xStepDirection)
					{
						/* first go to position 3588750 */
						xWorkTaskResponse = eWork_Started;
						/* TODO: Change form Quick'n'Dirty to good style */
						while((eWork_Done != xWorkTaskResponse) && (eWork_Abort != xWorkTaskResponse))
						{
							xWorkTaskResponse = xMoveWork(xWorkTaskResponse,3588750,eMove_IgnoreLimitSwitch);
							osDelay(5);
						}
						/* move successful ? */
						if(eWork_Done == xWorkTaskResponse)
						{
							/* and then restart */
							/* TODO: CSCAN: calculate start and stop angle according to the current angle resolution */
							si32StartAngle = 3588750;
							si32StopAngle = 11250;
							xWorkTaskResponse = eWork_Started;
						}
						else
						{
							xSysMode = eSysMode_Idle;
						}
					}
					else
					{
						/* first go to position 0 */
						xWorkTaskResponse = eWork_Started;
						/* TODO: Change form Quick'n'Dirty to good style */
						while((eWork_Done != xWorkTaskResponse) && (eWork_Abort != xWorkTaskResponse))
						{
							xWorkTaskResponse = xMoveWork(xWorkTaskResponse,0,eMove_IgnoreLimitSwitch);
							osDelay(5);
						}
						/* move successful ? */
						if(eWork_Done == xWorkTaskResponse)
						{
							/* and then restart */
							/* TODO: CSCAN: calculate start and stop angle according to the current angle resolution */
							si32StartAngle = 0;
							si32StopAngle = 3577500;
							xWorkTaskResponse = eWork_Started;
						}
						else
						{
							xSysMode = eSysMode_Idle;
						}
					}
				}
				else
				{
					/* keep state */
					xSysMode = eSysMode_ContinousScan;
				}
			}
		}
		else if(eSysMode_StaticScan == xSysMode)
		{
			if(0 == u08SampleSensorRQ)
			{
				/* request sampling */
				SensorSamplingStart();
				/* sensor sampling request successful */
				u08SampleSensorRQ = 1;

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
						if(eCMD_SensorValueReady == xMqCmd.xMQ_Cmd)
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
		else if(eSysMode_MoveToAngle == xSysMode)
		{
			/* first check if an abort request was received */
			if(eCMD_Abort == xMqCOMCmd.xMQ_Cmd)
			{
				xSysMode = eSysMode_Idle;
			}
			else
			{
				xWorkTaskResponse = xMoveWork(xWorkTaskResponse, si32StopAngle, eMove_IgnoreLimitSwitch);
				if((eWork_Abort == xWorkTaskResponse) || (eWork_Done == xWorkTaskResponse))
				{
					/* error or done -> switch to idle mode */
					xSysMode = eSysMode_Idle;
				}
				else
				{
					/* keep state */
					xSysMode = eSysMode_MoveToAngle;
				}
			}
		}
		else if(eSysMode_Homing == xSysMode)
		{
			/* first check if an abort request was received */
			if(eCMD_Abort == xMqCOMCmd.xMQ_Cmd)
			{
				xSysMode = eSysMode_Idle;
			}
			else
			{
				xWorkTaskResponse = xMoveWork(xWorkTaskResponse, si32StopAngle, eMove_CheckLimitSwitch);
				if(eWork_LimitSwitchHit == xWorkTaskResponse)
				{
					/* set current position to zero */
					xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle = 0;
					xSysMode = eSysMode_Idle;
				}
				else if((eWork_Abort == xWorkTaskResponse) || (eWork_Done == xWorkTaskResponse))
				{
					/* error or done -> switch to idle mode */
					xSysMode = eSysMode_Idle;
				}
				else
				{
					/* keep state */
					xSysMode = eSysMode_Homing;
				}
			}
		}
		else if(eSysMode_TestMode == xSysMode)
		{
			for(uint16_t u16TestReg=0;u16TestReg < 4096;u16TestReg++)
			{
				Print_Test_Data_Value(u16TestReg);
				osDelay(5);
			}
			xSysMode = eSysMode_Idle;
		}
		else if(eSysMode_Idle == xSysMode)
		{
			/* evaluate incoming com message */
			switch (xMqCOMCmd.xMQ_Cmd) {
				case eCMD_FullScanMeasurement:
					xSysMode = eSysMode_FullScan;
					break;
				case eCMD_PartialScanMeasurement:
					/* take over parameters */
					si32StartAngle = xMqCOMCmd.i32MQ_Cmd_ParamB;
					si32StopAngle = xMqCOMCmd.i32MQ_Cmd_ParamC;
					su16SampleCount = (uint16_t)xMqCOMCmd.i32MQ_Cmd_ParamA;
					/* set state */
					xWorkTaskResponse = eWork_Started;
					/* change state */
					xSysMode = eSysMode_PartialScan;
					break;
				case eCMD_ContinousScanMeasurment:
					/* set parameters */
					/* TODO: CSCAN: calculate start and stop angle according to the current angle resolution */
					si32StartAngle = 0;
					si32StopAngle = 3577500;
					su16SampleCount = (uint16_t)xMqCOMCmd.i32MQ_Cmd_ParamA;
					/* set state */
					xWorkTaskResponse = eWork_Started;
					/* change system state */
					xSysMode = eSysMode_ContinousScan;
					break;
				case eCMD_MoveToAngle:
					si32StopAngle = xMqCOMCmd.i32MQ_Cmd_ParamA;
					xWorkTaskResponse = eWork_Started;
					xSysMode = eSysMode_MoveToAngle;
					break;
				case eCMD_StaticMeasurement:

					break;
				case eCMD_SampleSensor:
					u08SampleSensorRQ = 0;
					xSysMode = eSysMode_StaticScan;
					break;
				case eCMD_SetAngleResolution:
					xSTMD_Config.xSTMD_Rotation.i32STMD_AngleIncrement = xMqCOMCmd.i32MQ_Cmd_ParamA;
					xSysMode = eSysMode_Idle;
					break;
				case eCMD_SetStepResolution:
					if(eSTMD_StepRes_Coarse == xMqCOMCmd.i32MQ_Cmd_ParamA)
					{
						xSTMD_Config.xSTMD_Rotation.xStepResolution = eSTMD_StepRes_Coarse;
					}
					else
					{
						xSTMD_Config.xSTMD_Rotation.xStepResolution = eSTMD_StepRes_Fine;
					}
					setStepResolution(xSTMD_Config.xSTMD_Rotation.xStepResolution);
					break;
				case eCMD_PrintSTMDConfig:
					Print_Config();
					xSysMode = eSysMode_Idle;
					break;
				case eCMD_PerformHoming:
					/* preset current position */
					xSTMD_Config.xSTMD_Rotation.i32STMD_CurrentAngle = 450000;
					/* prepare move */
					si32StopAngle = 0;
					xWorkTaskResponse = eWork_Started;
					xSysMode = eSysMode_Homing;
					break;
				case eCMD_StartTestMode:
					xSysMode = eSysMode_TestMode;
					break;
				default:
					break;
			}
		}
	}
  /* USER CODE END 5 */ 
}

/* COMTaskWork function */
void COMTaskWork(void const * argument)
{
  /* USER CODE BEGIN COMTaskWork */
//   HAL_StatusTypeDef xRetVal = HAL_ERROR;
	uint8_t au08RxBuffer[20];
	uint8_t u08Idx;
//	volatile uint32_t u32Length;
//	volatile int8_t i08USBCDCErrCode;

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

/* SMTDWork function */
void SMTDWork(void const * argument)
{
  /* USER CODE BEGIN SMTDWork */
	T_MQ_Command xMqCommand;
	T_WorkStates xSTMDWorkState = eWork_Idle;
	int32_t i32TargetAngle = 0;
	/* for readability */
	T_STMD_Rotation * pxRotation = &xSTMD_Config.xSTMD_Rotation;
	T_Move_LimitSwitch xLimitSwitchBehaviour = eMove_IgnoreLimitSwitch;
	T_WorkStates xSteppingRetVal = eWork_Idle;

	volatile int32_t i32StepCount;

  /* Infinite loop */
  for(;;)
  {
    osDelay(25);
    /* preload in case of no message */
    xMqCommand.xMQ_Cmd = eCMD_None;
    /* new message available ? */
    if(0 != xQMainToSTMDHandle)
    {
    	xQueueReceive(xQMainToSTMDHandle,&xMqCommand,0);
    }

    /* work depending on state */
    switch (xSTMDWorkState) {
		case eWork_Idle:
			/* process any incoming command */
			if(eCMD_MoveToAngle == xMqCommand.xMQ_Cmd)
			{
				/* evaluate parameters */
				/* take over */
				i32TargetAngle = xMqCommand.i32MQ_Cmd_ParamA;
				xLimitSwitchBehaviour = xMqCommand.i32MQ_Cmd_ParamB;
				/* check, if moving is necessary */
				if(i32TargetAngle != pxRotation->i32STMD_CurrentAngle)
				{
					/* choose in which direction to step */
					if(i32TargetAngle > pxRotation->i32STMD_CurrentAngle)
					{
						pxRotation->xStepDirection = eSTMD_CW;
						/* calculate amount of steps needed to reach target */
						i32StepCount = (i32TargetAngle - pxRotation->i32STMD_CurrentAngle)/pxRotation->i32STMD_StepIncrement;
					}
					else if(i32TargetAngle < pxRotation->i32STMD_CurrentAngle)
					{
						pxRotation->xStepDirection = eSTMD_CCW;
						/* calculate amount of steps needed to reach target */
						i32StepCount = (pxRotation->i32STMD_CurrentAngle - i32TargetAngle)/pxRotation->i32STMD_StepIncrement;
					}
					/* and start moving */
					xSTMDWorkState = eWork_Moving;
				}
				else
				{
					/* already at the given position */
					xSTMDWorkState = eWork_Done;
				}
			}
			break;
		case eWork_Moving:
			/* first check if there is an abort request */
			if(eCMD_Abort != xMqCommand.xMQ_Cmd)
			{
				/* perform stepping */
				if(0 != i32StepCount)
				{
					//xSteppingRetVal = xSTMD_PerformStepping(pxRotation->xStepDirection, i32StepCount, eMove_IgnoreLimitSwitch);
					xSteppingRetVal = xSTMD_PerformStepping(pxRotation->xStepDirection, i32StepCount, xLimitSwitchBehaviour);
					if(eWork_Done == xSteppingRetVal )
					{
						/* reached position */
						xSTMDWorkState = eWork_Done;
					}
					else if(eWork_LimitSwitchHit == xSteppingRetVal)
					{
						/* do something */
						xSTMDWorkState = eWork_LimitSwitchHit;
					}
					else
					{
						xSTMDWorkState = eWork_Abort;
					}
				}
			}
			else
			{
				xSTMDWorkState = eWork_Abort;
			}
			break;
		case eWork_Done:
			/* inform main task */
			if(0 != xQSTMDToMainHandle)
			{
				xMqCommand.xMQ_Cmd = eCMD_Accepted;
				xQueueSendToBack(xQSTMDToMainHandle,&xMqCommand,0);
			}
			xSTMDWorkState = eWork_Idle;
			break;
		case eWork_Abort:
			/* inform main task */
			if(0 != xQSTMDToMainHandle)
			{
				xMqCommand.xMQ_Cmd = eCMD_Rejected;
				xQueueSendToBack(xQSTMDToMainHandle,&xMqCommand,0);
			}
			xSTMDWorkState = eWork_Idle;
			break;
		case eWork_LimitSwitchHit:
			/* inform main task */
			if(0 != xQSTMDToMainHandle)
			{
				xMqCommand.xMQ_Cmd = eCMD_LimitSwitchHit;
				xQueueSendToBack(xQSTMDToMainHandle,&xMqCommand,0);
			}
			xSTMDWorkState = eWork_Idle;
			break;
		default:
			break;
	}
    /* */
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
