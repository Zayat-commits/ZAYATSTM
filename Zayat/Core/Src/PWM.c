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
/* In this file:
 *
 * 		Making a function that generates a Pulse wave with duty cycle of choice.
 *
 * 		-The timer Register is 16 bits, only 10 bits are used.
 * 		-The clock frequency is 16 MHz, with a prescale of 1024
 *
 * 	The Goal is:
 *
 * 		-To produce a 50-60 Hz square wave with duty cycle of choice to control ESC.
 *
 * 	The laws governing the resulting frequency is:
 *
 * 		F = 16000000/(prescale * (1 + TOP))	; (where TOP is 2^10 - 1) in this case (10 bits) and prescale is 1024, but
 * 												we will not use max number.
 * 		16000000/(1024*(1+TOP) = 55 Hz for example, solve for TOP = 283
 * 		since we need only 283 counts and the timer full count is 1024, therefore we should set default starting by
 * 		1024 - 1 - 283 = 740.
 * 		Now, the timer will count from 740 to 283 then overflow, which gives us 283 counts, and thus produces
 * 		 55 Hz.
 *
 * 		To set duty cycle:
 *
 * 			-Place compare value in
 */
#include "PWM.h"
#define ONE_MS 1200
#define TWO_MS 2400

extern TIM_HandleTypeDef htim2;
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
		string_receive(buffer);
	}while(atoi(buffer) != 1 && atoi(buffer) != 2);
	switch (atoi(buffer))
		{
	case 1:
		parameter.status.pwm = PWM_ON;
		PWM(MIN,1);
		PWM(MIN,2);
		PWM(MIN,3);
		PWM(MIN,4);
		do
		{
			fview(PRINT_NORMAL, 0, "PWM = 0, INSERT 0# AFTER TONE (BEEP-BEEP-BEEP) OR REPLUG/RESET IN CASE OF NO TONE\n");
			string_receive(buffer);
		}while(atoi(buffer) != 0);
		parameter.status.armed = 1;
		fview(PRINT_NORMAL, 0, "----------------------------------------RECEIVED AND DONE ARMING------------------------------------------- \n");
		vdFrames("STARS");
		HAL_Delay(3000);
		parameter.ret_flag = 1;
	case 2:
		parameter.ret_flag = 1;
		break;
		}
}
	/*
	 *
	 * 		-Motor 1 = CHANNEL_1						1    2
	 * 		-Motor 2 = CHANNEL_1						 \  /
	 * 		-Motor 3 = CHANNEL_1						  []
	 * 		-Motor 4 = CHANNEL_1						 /  \
	 * 													3    4
	 *
	 */
void PWM(u32 dutyCycle, u8 motorNumber)
{
	u32 temp = dutyCycle * ONE_MS / 100 + ONE_MS;
	if(motorNumber == 1)parameter.motor1 = temp;
	if(motorNumber == 2)parameter.motor2 = temp;
	if(motorNumber == 3)parameter.motor3 = temp;
	if(motorNumber == 4)parameter.motor4 = temp;
	if(parameter.status.pwm == PWM_ON)
	{
		sConfigOCZayat.Pulse = temp;
		switch(motorNumber)
		{
		case 1:
			   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCZayat, TIM_CHANNEL_1) != HAL_OK)
			   {
				 Error_Handler();
			   }
			   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			   break;
		case 2:
			   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCZayat, TIM_CHANNEL_2) != HAL_OK)
			   {
				 Error_Handler();
			   }
			   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			   break;
		case 3:
			   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCZayat, TIM_CHANNEL_3) != HAL_OK)
			   {
				 Error_Handler();
			   }
			   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
			   break;
		case 4:
			   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCZayat, TIM_CHANNEL_4) != HAL_OK)
			   {
				 Error_Handler();
			   }
			   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
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
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
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
			string_receive(buffer);
		}while(atoi(buffer) != 1 && atoi(buffer) != 2);


		if(atoi(buffer) == 1)
			{
			parameter.status.pwm = PWM_ON;
			PWM(MAX,MOTOR1);
			PWM(MAX,MOTOR2);
			PWM(MAX,MOTOR3);
			PWM(MAX,MOTOR4);
			do
			{
				fview(PRINT_NORMAL, 0, "PWM = 100, INSERT 0 AFTER FIRST TONE (BEEP-BEEP) OR REPLUG/RESET IN CASE OF NO TONE\n");
				string_receive(buffer);
			}while(atoi(buffer) != 0);

			fview(PRINT_NORMAL, 0, "----------------------------------------------RECEIVED----------------------------------------------------- \n \n");
			PWM(MIN,MOTOR1);
			PWM(MIN,MOTOR2);
			PWM(MIN,MOTOR3);
			PWM(MIN,MOTOR4);
			do
			{
				fview(PRINT_NORMAL, 0, "PWM = 0, INSERT 0 AFTER SECOND TONE (BEEP-BEEP) OR REPLUG/RESET SYSTEM IN CASE OF NO TONE\n");
				string_receive(buffer);
			}while(atoi(buffer) != 0);

			fview(PRINT_NORMAL, 0, "-------------------------------------RECEIVED AND DONE CALIBRATION----------------------------------------- \n \n");
			parameter.status.calibrated = 1;
			parameter.ret_flag = 1;
			HAL_Delay(3000);
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
		fview(PRINT_NORMAL, 0, "USER NEEDS TO CALIBRATE/ARM THE MOTORS FIRST \n");
	}
	while(parameter.ret_flag == 0)
	{
		vdFrames("STARS");
		fview(PRINT_NORMAL, 0, "----------------------------------FREE CONTROL OF MOTOR SPEEDS UI------------------------------------------ \n \n");
		vdFrames("STARS");
		insert:

		fview(PRINT_NORMAL, 0, "TO RETURN TO PREVIOUS MENU INSERT -1 \nOTHERWISE, THE PWM RANGES FROM 0 ~ 100. \nINSERT HERE: ");
		string_receive(buffer);
		while(atoi(buffer) > 100 || atoi(buffer) < -1)
		{
			fview(PRINT_NORMAL, 0, "\nINSERT CORRECT NUMBER: ");
			string_receive(buffer);

		}
		if(atoi(buffer) == -1)
		{
			parameter.ret_flag = 1;

		}
		else
		{
			int speed = atoi(buffer);
			fview(PRINT_INT_NO_TAB, speed, "\t");
			fview(PRINT_NORMAL, 0, "CONTINUE? \n1:YES \t2: NO \t 0: RETURN TO MAIN MENU \n");
			string_receive(buffer);
			s8 temp = atoi(buffer);
			while((temp < 0 || temp > 2))
			{
				fview(PRINT_NORMAL, 0, "\nINSERT CORRECT NUMBER:\r ");
				string_receive(buffer);

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
				HAL_Delay(3000);
				break;
			default:
				goto insert;
			}
		}
	}
}
