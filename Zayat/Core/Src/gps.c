/*
 * gps.c
 *
 *  Created on: May 31, 2020
 *      Author: Khaled Ali
 */
#include "gps.h"
#include "main.h"
#include "STD_TYPES.h"
#include "main.h"
accel distance, speed;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
uint8_t gps_init_9600[37]= {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
		0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01,
		0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0x42, 0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22};
uint8_t gps_init_115200 [79]= {0xB5 ,0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, 0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30,
		0xB5, 0x62, 0x06, 0x01, 0x08 ,0x00 ,0x01 ,0x06 ,0x00 ,0x01 ,0x00 ,0x00 ,0x00 ,0x00 ,0x17 ,0xDA ,0xB5 ,0x62 ,0x06 ,0x01 ,0x02 ,0x00 ,0x01 ,0x06 ,0x10 ,0x39,
		0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF, 0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x09, 0x17, 0x40};
uint8_t gps_data[60];

void gps_init(void)
{
	  HAL_UART_Transmit(&huart2, gps_init_9600, strlen((char*)gps_init_9600), HAL_MAX_DELAY);
	  huart2.Init.BaudRate = 115200;
	  HAL_UART_Init(&huart2);
	  HAL_UART_Transmit(&huart2, gps_init_115200, strlen((char*)gps_init_115200), HAL_MAX_DELAY);
	  HAL_UART_Receive_DMA(&huart2, gps_data, 60);
}


uint8_t Read_gps()
{	static u16 counter=0;
int32_t position[3];
int32_t velocity[3];
uint8_t flag;
			flag = gps_data[16]; //Gps fix (PASS IT ONE I GET THE VARIABLE)
			position[0] = ((((int32_t)(gps_data[18])& 0xFF)) | (((int32_t)(gps_data[19]& 0xFF)) <<8)|(((int32_t)(gps_data[20]& 0xFF))<<16) | (((int32_t)(gps_data[21]& 0xFF))<<24))/100.0;
			position[1] = ((((int32_t)(gps_data[22])& 0xFF)) | (((int32_t)(gps_data[23]& 0xFF)) <<8)|(((int32_t)(gps_data[24]& 0xFF))<<16) | (((int32_t)(gps_data[25]& 0xFF))<<24))/100.0;
			position[2] = ((((int32_t)(gps_data[26])& 0xFF)) | (((int32_t)(gps_data[27]& 0xFF)) <<8)|(((int32_t)(gps_data[28]& 0xFF))<<16) | (((int32_t)(gps_data[29]& 0xFF))<<24))/100.0;
			velocity[0] = ((((int32_t)(gps_data[34])& 0xFF)) | (((int32_t)(gps_data[35]& 0xFF)) <<8)|(((int32_t)(gps_data[36]& 0xFF))<<16) | (((int32_t)(gps_data[37]& 0xFF))<<24))/100.0;
			velocity[1] = ((((int32_t)(gps_data[38])& 0xFF)) | (((int32_t)(gps_data[39]& 0xFF)) <<8)|(((int32_t)(gps_data[40]& 0xFF))<<16) | (((int32_t)(gps_data[41]& 0xFF))<<24))/100.0;
			velocity[2] = ((((int32_t)(gps_data[42])& 0xFF)) | (((int32_t)(gps_data[43]& 0xFF)) <<8)|(((int32_t)(gps_data[44]& 0xFF))<<16) | (((int32_t)(gps_data[45]& 0xFF))<<24))/100.0;
			if(counter==10000){
			fview(PRINT_FLOAT_WITH_TAB, position[0], "posx:");
			fview(PRINT_FLOAT_WITH_TAB, position[1], "posy:");
			fview(PRINT_FLOAT_WITH_TAB, position[2], "posz:");
			fview(PRINT_FLOAT_WITH_TAB, velocity[0], "velx:");
			fview(PRINT_FLOAT_WITH_TAB, velocity[1], "vely:");
			fview(PRINT_FLOAT_NO_TAB, velocity[2], "velz:");
			counter=0;
			}
			counter++;
		return flag;
}
