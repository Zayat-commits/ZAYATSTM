/*
 * bmp.c
 *
 *  Created on: May 20, 2020
 *      Author: Khaled Ali
 */
#include "main.h"
#include "math.h"
#include "bmp.h"
extern I2C_HandleTypeDef hi2c1;
uint8_t buf[30];
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t  ac4, ac5, ac6;
int32_t Up;
int32_t b5, Ut;
float temp, pressure;
void BMP_Init(void)
{
	volatile HAL_StatusTypeDef ret;
//	while(1)
	{
	HAL_Delay(150);
	buf[0]=0xAA;

	ret = HAL_I2C_IsDeviceReady(&hi2c1, 0xEE, 1000, 1000);
	ret = HAL_I2C_Master_Transmit(&hi2c1, (0x77 << 1), buf, 1, HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Receive(&hi2c1, (0x77 << 1), buf, 22, HAL_MAX_DELAY);
	ac1 = ((uint16_t)buf[0]<<8) | (buf[1]);
	ac2 = ((uint16_t)buf[2]<<8) | (buf[3]);
	ac3 = ((uint16_t)buf[4]<<8) | (buf[5]);
	ac4 = ((int16_t)buf[6]<<8) | (buf[7]);
	ac5 = ((int16_t)buf[8]<<8) | (buf[9]);
	ac6 = ((int16_t)buf[10]<<8)| (buf[11]);
	b1  = ((uint16_t)buf[12]<<8) | (buf[13]);
	b2  = ((uint16_t)buf[14]<<8) | (buf[15]);
	mb  = ((uint16_t)buf[16]<<8) | (buf[17]);
	mc  = ((uint16_t)buf[18]<<8) | (buf[19]);
	md  = ((uint16_t)buf[20]<<8) | (buf[21]);
	}
}
void UT(void)
{
	buf[0] = 0xF4;
	buf[1] = 0x2E;
	HAL_I2C_Master_Transmit(&hi2c1, 0xEE, buf, 2, HAL_MAX_DELAY);
	osDelay(5);
	buf[0] = 0xF6;
	HAL_I2C_Master_Transmit(&hi2c1, 0xEE, buf, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0xEE, buf, 2, HAL_MAX_DELAY);
	Ut = ((int16_t)buf[0]<<8) | (buf[1]);
}
void UP(void)
{
	buf[0] = 0xF4;
	buf[1] = 0x34 + (1<<6);
	HAL_I2C_Master_Transmit(&hi2c1, 0xEE, buf, 2, HAL_MAX_DELAY);
	osDelay(10);
	buf[0] = 0xF6;
	HAL_I2C_Master_Transmit(&hi2c1, 0xEE, buf, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0xEE, buf, 3, HAL_MAX_DELAY);
	Up = ((int16_t)buf[0]<<8) | (buf[1]);
	Up <<= 8;
	Up |= buf[2];
	Up	>>= 7;
}

float height(void)
{
	int32_t x1 = ((Ut -ac6) * (int32_t)ac5) >> 15;
    int32_t x2 = ((int32_t)mc << 11) / (x1 +md);
    b5 = x1 + x2;
    //temp = (int32_t)((b5 + 8) >> 4);
	int32_t b6 = b5 - 4000;
	x1 = ((int32_t)b2 * (b6 * b6 >> 12)) >> 11;
	x2 = (int32_t)ac2 * b6 >> 11;
	int32_t x3 = x1 + x2;
	int32_t b3 = ((((int32_t)ac1 * 4 + x3) << 1) + 2) >> 2;
	x1 = (int32_t) ac3 * b6 >> 13;
	x2 = ((int32_t)b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	uint32_t b4 = (uint32_t)ac4 * (x3 + 32768) >> 15;
	uint32_t b7 = ((Up - b3) * (50000 >> 1));
	int32_t p;
    if (b7 < 0x80000000)
        p = ((b7 * 2) / b4);
    else
        p = ((b7 / b4) * 2);
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);
    return 44330 * (1 - pow((double) p / 101325, 1/5.255));
}
