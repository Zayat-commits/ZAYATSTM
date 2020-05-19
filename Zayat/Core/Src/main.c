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
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STD_TYPES.h"
#include "Constants.h"
#include "PWM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
void string_receive(s8 buffer[]);
/* USER CODE BEGIN PFP */
void vInitPARAMETERS(parameters *ptr);
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
  vInitPARAMETERS(&parameter);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BODY_RATES */
  BODY_RATESHandle = osThreadNew(BodyRate, (void*) p, &BODY_RATES_attributes);

  /* creation of DRONE_START */
  DRONE_STARTHandle = osThreadNew(DroneStart, (void*) p, &DRONE_START_attributes);

  /* creation of IMU */
  IMUHandle = osThreadNew(MPU, (void*) p, &IMU_attributes);

  /* creation of PRINT_TTL */
  PRINT_TTLHandle = osThreadNew(PrintPARAMS, (void*) p, &PRINT_TTL_attributes);

  /* creation of INSERT_PARAMETE */
  INSERT_PARAMETEHandle = osThreadNew(insertPARAMS, (void*) p, &INSERT_PARAMETE_attributes);

  /* creation of OUTPUT_THRUST */
  OUTPUT_THRUSTHandle = osThreadNew(outputTHRUST, (void*) p, &OUTPUT_THRUST_attributes);

  /* creation of ROLL_PITCH */
  ROLL_PITCHHandle = osThreadNew(RollPitch, (void*) p, &ROLL_PITCH_attributes);

  /* creation of YAW */
  YAWHandle = osThreadNew(YawCONTROLLER, (void*) p, &YAW_attributes);

  /* creation of ALTITUDE_CONTRO */
  ALTITUDE_CONTROHandle = osThreadNew(Altitude, (void*) p, &ALTITUDE_CONTRO_attributes);

  /* creation of LATERAL_CONTROL */
  LATERAL_CONTROLHandle = osThreadNew(lateral, (void*) p, &LATERAL_CONTROL_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
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
  huart2.Init.BaudRate = 115200;
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

}

/* USER CODE BEGIN 4 */
void string_receive(s8* buffer)
{
	int i = 0;
	HAL_UART_Receive(&huart1, &buffer[i], 1, HAL_MAX_DELAY);
	while(buffer[i]!='#')
	{
		i++;
		HAL_UART_Receive(&huart1, &buffer[i], 1, HAL_MAX_DELAY);
	}
	buffer[i] = '\0';
}
void vInitPARAMETERS(parameters *ptr)
{
	/***********************************/
	/*TO INITIALIZE ALL PARAMETERS TO 0*/
	/***********************************/
	ptr->x = ptr->y = ptr->z = 0;
	ptr->x_dot = ptr->y_dot = ptr->z_dot = 0;
	ptr->x_dot_dot = ptr->y_dot_dot = ptr->z_dot_dot = 0;
	ptr->phi = ptr->theta = ptr->psi = 0;
	ptr->phi_dot = ptr->theta_dot = ptr->psi_dot = 0;
	ptr->p = ptr->q = ptr->r = 0;
	ptr->p_dot = ptr->q_dot = ptr->r_dot = 0;
	ptr->x_cmd = ptr->y_cmd = ptr->z_cmd = 0;
	ptr->x_dot_cmd = ptr->y_dot_cmd = ptr->z_dot_cmd = 0;
	ptr->x_dot_dot_cmd = ptr->y_dot_dot_cmd = ptr->z_dot_dot_cmd = 0;
	ptr->psi_cmd = ptr->p_cmd = ptr->q_cmd = ptr->r_cmd = 0;
	ptr->u1 = ptr->u2 = ptr->u3 = ptr->u4 = 0;
	ptr->cmd_thrust[0] = 0;
	ptr->cmd_thrust[1] = 0;
	ptr->cmd_thrust[2] = 0;
	ptr->cmd_thrust[3] = 0;
	ptr-> phib=ptr-> thetab=ptr-> psib=0;
}
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
	parameters* ptr = argument;
  /* Infinite loop */

  for(;;)
  {
		f32 p_error,q_error,r_error;
		p_error = ptr->p_cmd - ptr->p;
		q_error = ptr->q_cmd - ptr->q;
		r_error = ptr->r_cmd - ptr->r;
		ptr->p_dot = kp_p * p_error;
		ptr->q_dot = kp_q * q_error;
		ptr->r_dot = kp_r * r_error;
		ptr->u2 = Ixx * ptr->p_dot;
		ptr->u3 = Iyy * ptr->q_dot;
		ptr->u4 = Izz * ptr->r_dot;

		osDelay(20);
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

	/*TO CALIBRATE DRONE MOTOR OR START*/
	u8 buffer[20] = {"Calibrate = 0#\nDirect Start = 1#\n"};
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	HAL_UART_Receive(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);

  /* Infinite loop */
  for(;;)
  {/*
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
	default: vCalibrate_Motors();
		}
	vTaskDelete(NULL);
	*/
    osDelay(1);
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
  for(;;)
  {
    osDelay(1);
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
	s8 buffer[20];
	parameters *ptr = argument;
  /* Infinite loop */
  for(;;)
  {
		/***********************************/
		/*TO READ FORCE VALS IN WORLD FRAME*/
		/***********************************/

sprintf(buffer, "Value of F1 = %f\t", ptr->cmd_thrust[0]);
HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);

sprintf(buffer, "Value of F2 = %f\t", ptr->cmd_thrust[1]);
HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);

sprintf(buffer, "Value of F3 = %f\n", ptr->cmd_thrust[2]);
HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);

sprintf(buffer, "Value of F4 = %f\n", ptr->cmd_thrust[3]);
HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);

/***********************************/
/*TO READ IMU ANGLES IN WORLD FRAME*/
/***********************************/

//		dtostrf( ptr->phi, 7, 5, float_ );
//		sprintf(buffer, "Value of phi = %s\t", float_);
//		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
//
//		dtostrf( ptr->theta, 7, 5, float_ );
//		sprintf(buffer, "Value of theta = %s\t", float_);
//		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
//
//		dtostrf( ptr->psi, 7, 5, float_ );
//		sprintf(buffer, "Value of psi = %s\n", float_);
//		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);

/***********************************/
/*TO READ IMU ACCELS IN WORLD FRAME*/
/***********************************/

//		dtostrf( ptr->x_dot_dot, 7, 5, float_ );
//		sprintf(buffer, "Value of xdotdot = %s\t", float_);
//		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
//
//		dtostrf( ptr->y_dot_dot, 7, 5, float_ );
//		sprintf(buffer, "Value of ydotdot = %s\t", float_);
//		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
//
//		dtostrf( ptr->z_dot_dot, 7, 5, float_ );
//		sprintf(buffer, "Value of zdotdot = %s\n", float_);
//		HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	osDelay(40);
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
	parameters *ptr = argument;
	s8 buffer[20];
	s8 ok[] = {"ok"};
  /* Infinite loop */
  for(;;)
  {
		/*TO INSERT X Y Z TARGET TO PID BLK*/
	strcpy(buffer, "Insert x\n");
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	string_receive(buffer);
	HAL_UART_Transmit(&huart1, ok, strlen(ok), HAL_MAX_DELAY);

	ptr->x_cmd = atoi(buffer);
	strcpy(buffer, "Insert x dot\n");
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	string_receive(buffer);
	HAL_UART_Transmit(&huart1, ok, strlen(ok), HAL_MAX_DELAY);

	ptr->x_dot_cmd = atoi(buffer);
	strcpy(buffer, "Insert y\n");
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	string_receive(buffer);
	HAL_UART_Transmit(&huart1, ok, strlen(ok), HAL_MAX_DELAY);

	ptr->y_cmd = atoi(buffer);
	strcpy(buffer, "Insert y dot\n");
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	string_receive(buffer);
	HAL_UART_Transmit(&huart1, ok, strlen(ok), HAL_MAX_DELAY);

	ptr->y_dot_cmd = atoi(buffer);
	strcpy(buffer, "Insert z\n");
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	string_receive(buffer);
	HAL_UART_Transmit(&huart1, ok, strlen(ok), HAL_MAX_DELAY);

	ptr->z_cmd = atoi(buffer);
	strcpy(buffer, "Insert z dot\n");
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	string_receive(buffer);
	HAL_UART_Transmit(&huart1, ok, strlen(ok), HAL_MAX_DELAY);

	ptr->z_dot_cmd = atoi(buffer);
	strcpy(buffer, "Insert psi\n");
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	string_receive(buffer);
	HAL_UART_Transmit(&huart1, ok, strlen(ok), HAL_MAX_DELAY);

	ptr->psi_cmd = atoi(buffer);
	osDelay(60000);
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
	parameters *ptr = argument;

  /* Infinite loop */
  for(;;)
  {

	f32 l = L/1.4142135623;
	f32 t1 = ptr->u2/ l;
	f32 t2 = ptr->u3 / l;
	f32 t3 = - ptr->u4/ k_thrust;
	f32 t4 = ptr->u1;
	ptr->cmd_thrust[0] = (t1 + t2 + t3 + t4)/4.f; // front left
	ptr->cmd_thrust[1] = (-t1 + t2 - t3 + t4)/4.f; // front right
	ptr->cmd_thrust[2] = (t1 - t2 - t3 + t4)/4.f ; // rear left
	ptr->cmd_thrust[3] = (-t1 - t2 + t3 + t4)/4.f; // rear right

	/*************/
	/*PWM MAPPING*/
	/*************/
	u8 i;
	u16 speed_pwm[4];
	for(i=0 ; i<4;i++)
	{
		if (ptr->cmd_thrust[i] < F_min) ptr->cmd_thrust[i]=F_min;
		if (ptr->cmd_thrust[i] > F_max) ptr->cmd_thrust[i]=F_max;
		speed_pwm[i] = (1/(F_max-F_min))*ptr->cmd_thrust[i]*500.0;
		PWM(speed_pwm[i],i+1);
	}
    osDelay(20);
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
	parameters* ptr = argument;
  /* Infinite loop */
  for(;;)
  {
	f32 b_x_dot_cmd, b_y_dot_cmd, taw=1/kp_bank;
	f32 R11 = 1;
	f32 R12 = sin(ptr->phi) * sin(ptr->theta) / cos(ptr->theta);
	f32 R13= cos(ptr->phi) * sin(ptr->theta) / cos(ptr->theta);
	f32 R21 = 0;
	f32 R22 = cos(ptr->phi);
	f32 R23= -sin(ptr->phi);
	f32 R31 = 0;
	f32 R32 = sin(ptr->phi) / cos(ptr->theta);
	f32 R33 = cos(ptr->phi) / cos(ptr->theta);
	f32 R13_cmd= ptr->x_dot_dot_cmd*m/ptr->u1;
	f32 R23_cmd= ptr->y_dot_dot_cmd*m/ptr->u1;
	b_x_dot_cmd= (R13-R13_cmd)/taw;
	b_y_dot_cmd= (R23-R23_cmd)/taw;
	ptr->p_cmd = 1/R33 * (R21*b_x_dot_cmd - R11*b_y_dot_cmd);
	ptr->q_cmd = 1/R33 * (R22*b_x_dot_cmd - R12*b_y_dot_cmd);

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
	parameters* ptr = argument;
  /* Infinite loop */
  for(;;)
  {
	ptr->r_cmd = kp_yaw*(ptr->psi_cmd - ptr->psi);

	osDelay(20);
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
	parameters* ptr = argument;
  /* Infinite loop */
  for(;;)
  {
	f32 R33 = cos(ptr->phi)/cos(ptr->theta);
	ptr->z_dot_dot_cmd= kp_z*(ptr->z_cmd- ptr->z) + kd_z*(ptr->z_dot_cmd-ptr->z_dot);
	ptr->u1 = m * (ptr->z_dot_dot_cmd - g)/R33;

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
	parameters* ptr = argument;
  /* Infinite loop */
  for(;;)
  {
	ptr->x_dot_dot_cmd= kp_xy*(ptr->x_cmd- ptr->z) + kd_xy*(ptr->x_dot_cmd-ptr->x_dot);
	ptr->y_dot_dot_cmd= kp_xy*(ptr->y_cmd- ptr->z) + kd_xy*(ptr->y_dot_cmd-ptr->y_dot);

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
