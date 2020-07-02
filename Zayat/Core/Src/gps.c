/*
 * gps.c
 *
 *  Created on: May 31, 2020
 *      Author: Khaled Ali
 */
#include "main.h"
accel gps_position_offset,gps_velocity,gps_position;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern parameters parameter;
uint8_t gps_init_9600[37]= {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
		0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01,
		0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0x42, 0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22};
uint8_t gps_init_115200 [69]= {0xB5 , 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, 0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30, 0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe1, 0xb5, 0x62, 0x06, 0x01, 0x02, 0x00, 0x01, 0x07, 0x11, 0x3a,0xb5,0x62,0x06,0x09,0x0d,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x1d,0xab};
uint8_t gps_data[92];

void gps_init(void)
{
	  HAL_UART_Transmit(&huart2, gps_init_9600, 37, HAL_MAX_DELAY);
	  huart2.Init.BaudRate = 115200;
	  HAL_UART_Init(&huart2);
	  HAL_UART_Transmit(&huart2, gps_init_115200, 69, HAL_MAX_DELAY);
	  HAL_UART_Receive_DMA(&huart2, gps_data, 92);
	  char strin[] = "waiting for fix\n";

	while(Read_gps(&gps_position,&gps_velocity)!=3){
		 HAL_UART_Transmit(&huart1,strin,strlen(strin),HAL_MAX_DELAY);
		 HAL_Delay(5);
	 }

	  HAL_Delay(1000); ////////////////////////////////////////////EDIT THIS//////////////////////////////////////////////////////////////////7
	  Read_gps(&gps_position,&gps_velocity);
	  gps_position_offset.x = gps_position.x; gps_position_offset.y = gps_position.y; gps_position_offset.z = gps_position.z;
	  parameter.status.gps_state = 1;
		fview(PRINT_FLOAT_WITH_TAB, gps_position_offset.x, "offsetx:");
		fview(PRINT_FLOAT_WITH_TAB, gps_position_offset.y, "offsety:");
		fview(PRINT_FLOAT_NO_TAB, gps_position_offset.z, "offsetz:");
}


uint8_t Read_gps(accel *position, accel *velocity)
{
	static int e=0;
	while(gps_data[e]!= 0xb5 && gps_data[e+1]!=0x62)
	{
	e++;
	if (e==92)e=0;
	}

uint8_t flag;
flag = gps_data[(26+e)%92]; //Gps fix (PASS IT ONE I GET THE VARIABLE)
flag = 3; //Gps fix (PASS IT ONE I GET THE VARIABLE)

			position->x = ((((int32_t)(gps_data[(30+e)%92])& 0xFF)) | (((int32_t)(gps_data[(31+e)%92]& 0xFF)) <<8)|(((int32_t)(gps_data[(32+e)%92]& 0xFF))<<16) | (((int32_t)(gps_data[(33+e)%92]& 0xFF))<<24))*111139.0/10000000;
			position->y = ((((int32_t)(gps_data[(34+e)%92])& 0xFF)) | (((int32_t)(gps_data[(35+e)%92]& 0xFF)) <<8)|(((int32_t)(gps_data[(36+e)%92]& 0xFF))<<16) | (((int32_t)(gps_data[(37+e)%92]& 0xFF))<<24))*111139.0/10000000;
			position->z = -((((int32_t)(gps_data[(42+e)%92])& 0xFF)) | (((int32_t)(gps_data[(43+e)%92]& 0xFF)) <<8)|(((int32_t)(gps_data[(44+e)%92]& 0xFF))<<16) | (((int32_t)(gps_data[(45+e)%92]& 0xFF))<<24))/1000.0;
			velocity->y = ((((int32_t)(gps_data[(54+e)%92])& 0xFF)) | (((int32_t)(gps_data[(55+e)%92]& 0xFF)) <<8)|(((int32_t)(gps_data[(56+e)%92]& 0xFF))<<16) | (((int32_t)(gps_data[(57+e)%92]& 0xFF))<<24))/1000.0;
			velocity->x = ((((int32_t)(gps_data[(58+e)%92])& 0xFF)) | (((int32_t)(gps_data[(59+e)%92]& 0xFF)) <<8)|(((int32_t)(gps_data[(60+e)%92]& 0xFF))<<16) | (((int32_t)(gps_data[(61+e)%92]& 0xFF))<<24))/1000.0;
			velocity->z = ((((int32_t)(gps_data[(62+e)%92])& 0xFF)) | (((int32_t)(gps_data[(63+e)%92]& 0xFF)) <<8)|(((int32_t)(gps_data[(64+e)%92]& 0xFF))<<16) | (((int32_t)(gps_data[(65+e)%92]& 0xFF))<<24))/1000.0;
			position->x -= gps_position_offset.x;
			position->y -= gps_position_offset.y;
			position->z -= gps_position_offset.z;
			position->x = 0;
			position->y = 0;
			position->z = 0; ///////////////////////////////////////////////////////////////
		return flag;
}
