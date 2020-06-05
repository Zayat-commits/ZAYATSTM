/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//im here
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for BODY_RATES */
osThreadId_t BODY_RATESHandle;
const osThreadAttr_t BODY_RATES_attributes = {
  .name = "BODY_RATES",
  .priority = (osPriority_t) osPriorityNormal7,
  .stack_size = 150 * 4
};
/* Definitions for DRONE_START */
osThreadId_t DRONE_STARTHandle;
const osThreadAttr_t DRONE_START_attributes = {
  .name = "DRONE_START",
  .priority = (osPriority_t) osPriorityNormal3,
  .stack_size = 300 * 4
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .priority = (osPriority_t) osPriorityAboveNormal2,
  .stack_size = 300 * 4
};
/* Definitions for PRINT_TTL */
osThreadId_t PRINT_TTLHandle;
const osThreadAttr_t PRINT_TTL_attributes = {
  .name = "PRINT_TTL",
  .priority = (osPriority_t) osPriorityNormal4,
  .stack_size = 300 * 4
};
/* Definitions for INSERT_PARAMETE */
osThreadId_t INSERT_PARAMETEHandle;
const osThreadAttr_t INSERT_PARAMETE_attributes = {
  .name = "INSERT_PARAMETE",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 300 * 4
};
/* Definitions for OUTPUT_THRUST */
osThreadId_t OUTPUT_THRUSTHandle;
const osThreadAttr_t OUTPUT_THRUST_attributes = {
  .name = "OUTPUT_THRUST",
  .priority = (osPriority_t) osPriorityNormal5,
  .stack_size = 128 * 4
};
/* Definitions for ROLL_PITCH */
osThreadId_t ROLL_PITCHHandle;
const osThreadAttr_t ROLL_PITCH_attributes = {
  .name = "ROLL_PITCH",
  .priority = (osPriority_t) osPriorityNormal7,
  .stack_size = 300 * 4
};
/* Definitions for YAW */
osThreadId_t YAWHandle;
const osThreadAttr_t YAW_attributes = {
  .name = "YAW",
  .priority = (osPriority_t) osPriorityNormal7,
  .stack_size = 128 * 4
};
/* Definitions for ALTITUDE_CONTRO */
osThreadId_t ALTITUDE_CONTROHandle;
const osThreadAttr_t ALTITUDE_CONTRO_attributes = {
  .name = "ALTITUDE_CONTRO",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for LATERAL_CONTROL */
osThreadId_t LATERAL_CONTROLHandle;
const osThreadAttr_t LATERAL_CONTROL_attributes = {
  .name = "LATERAL_CONTROL",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 129 * 4
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void BodyRate(void *argument);
void DroneStart(void *argument);
void MPU(void *argument);
void PrintPARAMS(void *argument);
void insertPARAMS(void *argument);
void outputTHRUST(void *argument);
void RollPitch(void *argument);
void YawCONTROLLER(void *argument);
void Altitude(void *argument);
void lateral(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TIM_OC_InitTypeDef sConfigOCZayat = {0};
parameters parameter;
parameters* p = &parameter;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  vInitPARAMETERS(&parameter);
//  HAL_Delay(150);
//  MPU_Init(p, INTEGRAL_DT);
//  Accel_calibration(p, INTEGRAL_DT);
//  Compass_Init();


  accel gps_position_offset,gps_velocity,gps_position;
  gps_init();
  while(Read_gps(&gps_position,&gps_velocity)!=3);
    gps_position_offset.x = gps_position.x; gps_position_offset.y = gps_position.y; gps_position_offset.z = gps_position.z;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
////////  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
////////  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
////////  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
////////  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
//  /* creation of BODY_RATES */
//  BODY_RATESHandle = osThreadNew(BodyRate, (void*) p, &BODY_RATES_attributes);
//
//  /* creation of DRONE_START */
//  DRONE_STARTHandle = osThreadNew(DroneStart, (void*) p, &DRONE_START_attributes);
//
//  /* creation of IMU */
//  IMUHandle = osThreadNew(MPU, (void*) p, &IMU_attributes);
//
//  /* creation of PRINT_TTL */
//  PRINT_TTLHandle = osThreadNew(PrintPARAMS, (void*) p, &PRINT_TTL_attributes);
//
//  /* creation of INSERT_PARAMETE */
//  INSERT_PARAMETEHandle = osThreadNew(insertPARAMS, (void*) p, &INSERT_PARAMETE_attributes);
//
//  /* creation of OUTPUT_THRUST */
//  OUTPUT_THRUSTHandle = osThreadNew(outputTHRUST, (void*) p, &OUTPUT_THRUST_attributes);
//
//  /* creation of ROLL_PITCH */
//  ROLL_PITCHHandle = osThreadNew(RollPitch, (void*) p, &ROLL_PITCH_attributes);
//
//  /* creation of YAW */
//  YAWHandle = osThreadNew(YawCONTROLLER, (void*) p, &YAW_attributes);
//
//  /* creation of ALTITUDE_CONTRO */
//  ALTITUDE_CONTROHandle = osThreadNew(Altitude, (void*) p, &ALTITUDE_CONTRO_attributes);
//
//  /* creation of LATERAL_CONTROL */
//  LATERAL_CONTROLHandle = osThreadNew(lateral, (void*) p, &LATERAL_CONTROL_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
////////////////  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
//  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
Read_gps(&gps_position,&gps_velocity);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 200000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 60;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 24000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  sConfigOCZayat = sConfigOC;
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


//GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//


/* USER CODE END 4 */

/* USER CODE BEGIN Header_BodyRate */
/**
  * @brief  Function implementing the BODY_RATES thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_BodyRate */
void BodyRate(void *argument)
{
  /* USER CODE BEGIN 5 */
//	parameters* ptr = argument;
  /* Infinite loop */

  for(;;)
  {

	  vdBodyRatesBlock(argument);
/*		f32 p_error,q_error,r_error;
		p_error = ptr->p_cmd - ptr->p;
		q_error = ptr->q_cmd - ptr->q;
		r_error = ptr->r_cmd - ptr->r;
		ptr->p_dot = kp_p * p_error;
		ptr->q_dot = kp_q * q_error;
		ptr->r_dot = kp_r * r_error;
		ptr->u2 = Ixx * ptr->p_dot;
		ptr->u3 = Iyy * ptr->q_dot;
		ptr->u4 = Izz * ptr->r_dot;*/

		osDelay(10);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_DroneStart */
/**
* @brief Function implementing the DRONE_START thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DroneStart */
void DroneStart(void *argument)
{
  /* USER CODE BEGIN DroneStart */
		char buffer[10];
	  /* Infinite loop */
		for(;;)
		{
			fview(PRINT_NORMAL, 0, "0 to Calibrate, 1 to ARM, 2 to insert speed \n");
			string_receive(buffer);
		switch (atoi(buffer))
			{
		case 0:
				vCalibrate_Motors();
				break;
		case 1:	ARM_Motors();
				PWM(ARMED,1);
				PWM(ARMED,2);
				PWM(ARMED,3);
				PWM(ARMED,4);
				break;
		case 2: fview(PRINT_NORMAL, 0, "insert speed, range = 0% -> 100%, -1 to exit\n");
				string_receive(buffer);
				while(atoi(buffer) != -1)
				{

					if(atoi(buffer) < 0 || atoi(buffer) > 100)
					{
						u8 error = 1;
						fview(PRINT_NORMAL, 0, "insert correct number (0 ~ 100) \n");
						while(error == 1)
						{
							string_receive(buffer);
							error = (atoi(buffer) < 0 || atoi(buffer) > 100)?  1 : 0;
						}
					}
					PWM(atoi(buffer),1);
					PWM(atoi(buffer),2);
					PWM(atoi(buffer),3);
					PWM(atoi(buffer),4);
					fview(PRINT_NORMAL, 0, "insert speed, range = 0% -> 100%, -1 to exit\n");
					string_receive(buffer);
				}
				break;
		default: vCalibrate_Motors();
			}
		vdDroneStartBlock(argument);
		osDelay(20);
	  }
  /* USER CODE END DroneStart */
}

/* USER CODE BEGIN Header_MPU */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPU */
void MPU(void *argument)
{
  /* USER CODE BEGIN MPU */
  /* Infinite loop */
	s32 tickzayat;
	for(;;)
	{
		tickzayat = osKernelGetTickCount();

		vdMPUBlock(argument);

		/*Calculate total ticks needed for 10 ms period*/
		tickzayat = osKernelGetTickCount() - tickzayat;
		tickzayat = 100 - tickzayat;
		if(tickzayat < 0)tickzayat = 100;
		tickzayat = osKernelGetTickCount() + tickzayat;
		/*---------------------------------------------*/
		osDelayUntil(tickzayat);
	}
  /* USER CODE END MPU */
}

/* USER CODE BEGIN Header_PrintPARAMS */
/**
* @brief Function implementing the PRINT_TTL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PrintPARAMS */
void PrintPARAMS(void *argument)
{
  /* USER CODE BEGIN PrintPARAMS */
  /* Infinite loop */
  for(;;)
  {
	  vdPrintBlock(argument);
	  osDelay(20);
  }
  /* USER CODE END PrintPARAMS */
}

/* USER CODE BEGIN Header_insertPARAMS */
/**
* @brief Function implementing the INSERT_PARAMETE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_insertPARAMS */
void insertPARAMS(void *argument)
{
  /* USER CODE BEGIN insertPARAMS */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END insertPARAMS */
}

/* USER CODE BEGIN Header_outputTHRUST */
/**
* @brief Function implementing the OUTPUT_THRUST thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_outputTHRUST */
void outputTHRUST(void *argument)
{
  /* USER CODE BEGIN outputTHRUST */
  /* Infinite loop */
  for(;;)
  {
	  vdOutputBlock(argument);
	  osDelay(10);
  }
  /* USER CODE END outputTHRUST */
}

/* USER CODE BEGIN Header_RollPitch */
/**
* @brief Function implementing the ROLL_PITCH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RollPitch */
void RollPitch(void *argument)
{
  /* USER CODE BEGIN RollPitch */
  /* Infinite loop */
  for(;;)
  {
	  vdRollPitchBlock(argument);
	  osDelay(20);
  }
  /* USER CODE END RollPitch */
}

/* USER CODE BEGIN Header_YawCONTROLLER */
/**
* @brief Function implementing the YAW thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_YawCONTROLLER */
void YawCONTROLLER(void *argument)
{
  /* USER CODE BEGIN YawCONTROLLER */
  /* Infinite loop */
  for(;;)
  {
	  vdYawBlock(argument);
	  osDelay(10);
  }
  /* USER CODE END YawCONTROLLER */
}

/* USER CODE BEGIN Header_Altitude */
/**
* @brief Function implementing the ALTITUDE_CONTRO thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Altitude */
void Altitude(void *argument)
{
  /* USER CODE BEGIN Altitude */
  /* Infinite loop */
  for(;;)
  {
	  vdAltitudeBlock(argument);
	  osDelay(20);
  }
  /* USER CODE END Altitude */
}

/* USER CODE BEGIN Header_lateral */
/**
* @brief Function implementing the LATERAL_CONTROL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lateral */
void lateral(void *argument)
{
  /* USER CODE BEGIN lateral */
  /* Infinite loop */
  for(;;)
  {
	  vdLateralBlock(argument);
	  osDelay(20);
  }
  /* USER CODE END lateral */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
