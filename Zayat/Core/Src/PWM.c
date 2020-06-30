/*
 * PWM.c
 *
 *  Created on: May 8, 2020
 *      Author: abdul
 */




/*
 * PWM.c
 *
 *  Created on: Feb 10, 2020
 *      Author: Zayat
 */

#include "PWM.h"
#define ONE_MS 1200
#define TWO_MS 2400

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern TIM_OC_InitTypeDef sConfigOCZayat;
extern parameters parameter;
void ARM_Motors(void)
{
	u8 buffer[10];

	vdFrames("STARS");
	fview(PRINT_NORMAL, 0, "-----------------------------ARE YOU SURE YOU WANT TO ARM THE MOTORS?-------------------------------------- \n \n");
	vdFrames("STARS");
	fview(PRINT_NORMAL, 0, "ARMING IS DONE BY APPLYING 0 % POWER TO MOTORS FOLLOWED BY 0  SPECIFIC TONE (BEEP-BEEP-BEEP)\n");
	do
	{
		fview(PRINT_NORMAL, 0, "1#: YES\t 2#: NO \n");
//		string_receive(buffer);
		HAL_UART_Receive(&huart1, buffer, 1, HAL_MAX_DELAY);
	}while(atoi(buffer) != 1 && atoi(buffer) != 2);
	switch (atoi(buffer))
		{
	case 1:
		parameter.status.pwm = PWM_ON;
		parameter.status.armed = 1;;
		PWM(MIN,MOTOR1);
		PWM(MIN,MOTOR2);
		PWM(MIN,MOTOR3);
		PWM(MIN,MOTOR4);
		do
		{
			fview(PRINT_NORMAL, 0, "PWM = 0, INSERT 0# AFTER TONE (BEEP-BEEP-BEEP) OR REPLUG/RESET IN CASE OF NO TONE\n");
//			string_receive(buffer);
			HAL_UART_Receive(&huart1, buffer, 1, HAL_MAX_DELAY);
		}while(atoi(buffer) != 0);
		parameter.status.armed = 1;
		fview(PRINT_NORMAL, 0, "----------------------------------------RECEIVED AND DONE ARMING------------------------------------------- \n");
		vdFrames("STARS");
		HAL_Delay(1000);
		parameter.ret_flag = 1;
	case 2:
		parameter.ret_flag = 1;
		break;
		}
}
	/*
	 *
	 * 		-Motor 1 = CHANNEL_1						1    2
	 * 		-Motor 2 = CHANNEL_2						 \  /
	 * 		-Motor 3 = CHANNEL_3						  []
	 * 		-Motor 4 = CHANNEL_4						 /  \
	 * 													3    4
	 *
	 */
void PWM(u32 dutyCycle, u8 motorNumber)
{
	u32 temp = dutyCycle  + ONE_MS;
	if(motorNumber == 1)parameter.motor1 = temp;
	if(motorNumber == 2)parameter.motor2 = temp;
	if(motorNumber == 3)parameter.motor3 = temp;
	if(motorNumber == 4)parameter.motor4 = temp;
	if(parameter.status.pwm == PWM_ON && (parameter.status.calibrated == 1 || parameter.status.armed == 1))
	{
		sConfigOCZayat.Pulse = temp;
		switch(motorNumber)
		{
		case 1:
			   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOCZayat, TIM_CHANNEL_1) != HAL_OK)
			   {
				 Error_Handler();
			   }
			   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			   break;
		case 2:
			   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOCZayat, TIM_CHANNEL_2) != HAL_OK)
			   {
				 Error_Handler();
			   }
			   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			   break;
		case 3:
			   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOCZayat, TIM_CHANNEL_3) != HAL_OK)
			   {
				 Error_Handler();
			   }
			   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			   break;
		case 4:
			   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOCZayat, TIM_CHANNEL_4) != HAL_OK)
			   {
				 Error_Handler();
			   }
			   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
			   break;
		default:
			PWM(0, MOTOR1);
			PWM(0, MOTOR2);
			PWM(0, MOTOR3);
			PWM(0, MOTOR4);
		}
	}
}

/* Disabling all timers*/
void DISARM_Motors(void)
{
	   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	   parameter.status.pwm = PWM_OFF;
//	   parameter.status.calibrated = 0;
//	   parameter.status.armed = 0;
}
void vCalibrate_Motors(void)
{
	u8 buffer[10];
	if(parameter.status.calibrated == 0 && parameter.status.pwm == PWM_OFF && parameter.status.armed == 0)
	{
		do
		{
			vdFrames("STARS");
			fview(PRINT_NORMAL, 0, "---------------------------------ARE YOU SURE YOU WANT TO CALIBRATE?---------------------------------------\n");
			vdFrames("STARS");
			fview(PRINT_NORMAL, 0, "CALIBRATION IS DONE BY APPLYING 100 % POWER TO MOTORS FOLLOWED BY 0 % SEPARATED BY SPECIFIC TONE (BEEP-BEEP)\n1: YES\t 2: NO \n");
//			string_receive(buffer);
			HAL_UART_Receive(&huart1, buffer, 2, HAL_MAX_DELAY);
		}while(atoi(buffer) != 1 && atoi(buffer) != 2);


		if(atoi(buffer) == 1)
			{
			parameter.status.pwm = PWM_ON;
			parameter.status.calibrated = 1;
			PWM(MAX,MOTOR1);
			PWM(MAX,MOTOR2);
			PWM(MAX,MOTOR3);
			PWM(MAX,MOTOR4);
			do
			{
				fview(PRINT_NORMAL, 0, "PWM = 100, INSERT 0 AFTER FIRST TONE (BEEP-BEEP) OR REPLUG/RESET IN CASE OF NO TONE\n");
//				string_receive(buffer);
				HAL_UART_Receive(&huart1, buffer, 2, HAL_MAX_DELAY);
			}while(atoi(buffer) != 0);

			fview(PRINT_NORMAL, 0, "----------------------------------------------RECEIVED----------------------------------------------------- \n \n");
			PWM(MIN,MOTOR1);
			PWM(MIN,MOTOR2);
			PWM(MIN,MOTOR3);
			PWM(MIN,MOTOR4);
			do
			{
				fview(PRINT_NORMAL, 0, "PWM = 0, INSERT 0 AFTER SECOND TONE (BEEP-BEEP) OR REPLUG/RESET SYSTEM IN CASE OF NO TONE\n");
//				string_receive(buffer);
				HAL_UART_Receive(&huart1, buffer, 2, HAL_MAX_DELAY);
			}while(atoi(buffer) != 0);

			fview(PRINT_NORMAL, 0, "-------------------------------------RECEIVED AND DONE CALIBRATION----------------------------------------- \n \n");
			parameter.status.calibrated = 1;
			parameter.ret_flag = 1;
			HAL_Delay(1000);
		}
		else if(atoi(buffer) == 2)
		{
			parameter.ret_flag = 1;
		}
	}
	else
	{
		fview(PRINT_NORMAL, 0, "FAILED TO CALIBRATE: PWM IS ON OR MOTORS ARE ARMED \n");
		parameter.ret_flag = 1;
		HAL_Delay(3000);
	}
}
void vdFreeRunPWM(void)
{
	s8 buffer[10];
	if(parameter.status.armed != 1 && parameter.status.calibrated != 1)
	{
		parameter.ret_flag = 1;
		fview(PRINT_NORMAL, 0, "USER NEEDS TO ARM THE MOTORS FIRST \n");
	}
	while(parameter.ret_flag == 0)
	{
		vdFrames("STARS");
		fview(PRINT_NORMAL, 0, "----------------------------------FREE CONTROL OF MOTOR SPEEDS UI------------------------------------------ \n \n");
		vdFrames("STARS");
		insert:

		fview(PRINT_NORMAL, 0, "TO RETURN TO PREVIOUS MENU INSERT -1 \nOTHERWISE, THE PWM RANGES FROM 0 ~ 100. \nINSERT HERE: ");
//		string_receive(buffer);
		HAL_UART_Receive(&huart1, buffer, 4, HAL_MAX_DELAY);

		while(atoi(buffer) > 100 || atoi(buffer) < -1)
		{
			fview(PRINT_NORMAL, 0, "\nINSERT CORRECT NUMBER: ");
//			string_receive(buffer);
			HAL_UART_Receive(&huart1, buffer, 4, HAL_MAX_DELAY);

		}
		if(atoi(buffer) == -1)
		{
			parameter.ret_flag = 1;

		}
		else
		{
			int speed = atoi(buffer) * MAX / 100;
			fview(PRINT_INT_NO_TAB, speed, "\t");
			fview(PRINT_NORMAL, 0, "CONTINUE? \n1:YES \t2: NO \t 0: RETURN TO MAIN MENU \n");
//			string_receive(buffer);
			HAL_UART_Receive(&huart1, buffer, 2, HAL_MAX_DELAY);

			while((atoi(buffer) < 0 || atoi(buffer) > 2))
			{
				fview(PRINT_NORMAL, 0, "\nINSERT CORRECT NUMBER:\r ");
//				string_receive(buffer);
				HAL_UART_Receive(&huart1, buffer, 2, HAL_MAX_DELAY);

			}
			switch (atoi(buffer))
			{
			case 0:
				parameter.ret_flag = 1;
				break;
			case 1:
				parameter.status.pwm = PWM_ON;
				PWM(speed, MOTOR1);
				PWM(speed, MOTOR2);
				PWM(speed, MOTOR3);
				PWM(speed, MOTOR4);
				fview(PRINT_NORMAL, 0, "----------------------------------------------RECEIVED----------------------------------------------------- \n");
				HAL_Delay(1000);
				break;
			default:
				goto insert;
			}
		}
	}
}
