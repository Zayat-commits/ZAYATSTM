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

void ARM_Motors(void)
{
	sConfigOCZayat.Pulse = 1.2 * ONE_MS;
   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCZayat, TIM_CHANNEL_1) != HAL_OK)
   {
	 Error_Handler();
   }
   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCZayat, TIM_CHANNEL_2) != HAL_OK)
   {
	 Error_Handler();
   }
   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCZayat, TIM_CHANNEL_3) != HAL_OK)
   {
	 Error_Handler();
   }
   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCZayat, TIM_CHANNEL_4) != HAL_OK)
   {
	 Error_Handler();
   }
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

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
void PWM(f32 dutyCycle, u8 motorNumber)
{
	f32 temp = dutyCycle * ONE_MS / 100 + ONE_MS;
	sConfigOCZayat.Pulse = (uint32_t) temp;

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
		ARM_Motors();
	}
}

/* Disabling all timers*/
void DISARM_Motors(void)
{
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	   HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
}
void vCalibrate_Motors(void)
{
	char buffer[50];
	PWM(MAX,1);
	PWM(MAX,2);
	PWM(MAX,3);
	PWM(MAX,4);

	fview(PRINT_NORMAL, 0, "Enter 0 again after First beep-beep\n");

	string_receive(buffer);

	while(atoi(buffer)!= 0);

	fview(PRINT_NORMAL, 0, "Received\n");

	PWM(MIN,1);
	PWM(MIN,2);
	PWM(MIN,3);
	PWM(MIN,4);

	fview(PRINT_NORMAL, 0, "Enter 0 again after First beep-beep\n");

	string_receive(buffer);

	while(atoi(buffer)!= 0);

	fview(PRINT_NORMAL, 0, "Received\n");
}
