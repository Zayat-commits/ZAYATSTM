

#include "compass.h"





f32 xMax, yMax, xMin, yMin,zMin, zMax;
f32     Mag_x_offset,    Mag_y_offset ,   Mag_z_offset ;
accel mag;
u8 buf[10];
int16_t temp1,temp2,temp3;

void Compass_Init()
{

	HAL_Delay(150);


//	buf[0]=0x37; //Bypassing
//	buf[1]=0x02;
//	HAL_I2C_Master_Transmit(&hi2c1, 0b11010000, buf, 2, HAL_MAX_DELAY);



	xMax= yMax= xMin= yMin=zMax= zMin=0.0;


	buf[0]=0x00; //initialising the compass
	buf[1]=0x70;
	buf[2]=0xA0;
	buf[3]=0x00;
	HAL_StatusTypeDef ret;
	//ret=HAL_I2C_Master_Transmit(&hi2c1, 0x3C, buf, 4, HAL_MAX_DELAY);
	ret = HAL_I2C_IsDeviceReady(&hi2c1, 0x3C, 1, HAL_MAX_DELAY);
	while(ret !=HAL_OK)
	{
		HAL_I2C_DeInit(&hi2c1);
		//HAL_Delay(10);
		HAL_I2C_Init(&hi2c1);
		//HAL_Delay(10);
		ret = HAL_I2C_IsDeviceReady(&hi2c1, 0x3C, 1, HAL_MAX_DELAY);
	}




HAL_Delay(100);


//		buf[0]=0x03;
//		HAL_I2C_Master_Transmit(&hi2c1, 0x3C, buf, 1, HAL_MAX_DELAY);
//		HAL_I2C_Master_Receive(&hi2c1, 0x3C, buf, 6, HAL_MAX_DELAY);
//
//					temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
//					temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
//					temp3 = ((int16_t)buf[4]<<8) | (buf[5]);
//
//					mag.x = temp1/1.0f;
//
//					mag.z = temp2/1.0f;
//
//					mag.y = temp3/1.0f;


//
//	for (u16 i=0 ; i<12000 ; i++)
//		{
//
//		if (xMin<mag.x) xMin=mag.x;
//		if (xMax>mag.x) xMax=mag.x;
//		if (yMin<mag.y) yMin=mag.y;
//		if (yMax>mag.y) yMax=mag.y;
//		if (zMin<mag.z) zMin=mag.z;
//		if (zMax>mag.z) zMax=mag.z;
//
//		buf[0]=0x03;
//		HAL_I2C_Master_Transmit(&hi2c1, 0x3C, buf, 1, HAL_MAX_DELAY);
//		HAL_I2C_Master_Receive(&hi2c1, 0x3C, buf, 6, HAL_MAX_DELAY);
//
//					temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
//					temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
//					temp3 = ((int16_t)buf[4]<<8) | (buf[5]);
//
//					mag.x = temp1/1.0f;
//
//					mag.z = temp2/1.0f;
//
//					mag.y = temp3/1.0f;
//
//				HAL_Delay(5);
//		}
//	fview(PRINT_FLOAT_NO_TAB, xMax, "Value of xmx = ");
//	fview(PRINT_FLOAT_NO_TAB, xMin, "Value of xmin = ");
//	fview(PRINT_FLOAT_NO_TAB, yMax, "Value of ymax = ");
//	fview(PRINT_FLOAT_NO_TAB, yMin, "Value of ymin = ");
//	fview(PRINT_FLOAT_NO_TAB, zMax, "Value of zmax = ");
//	fview(PRINT_FLOAT_NO_TAB, zMin, "Value of zmin = ");

	xMax=  -661; xMin=694; yMax= -207; yMin=876; zMax= -724;zMin=847;

//send smth like DONE CALIB


}

void Read_Compass_Values(parameters *body)
{
HAL_StatusTypeDef ret;

	buf[0]=3;
	int counterzayat = 12;
	ret = HAL_I2C_IsDeviceReady(&hi2c1, 0x3C, 1, HAL_MAX_DELAY);
	while(ret !=HAL_OK)
	{
		HAL_I2C_Master_Abort_IT(&hi2c1, 0x3C);
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret = HAL_I2C_IsDeviceReady(&hi2c1, 0x3C, 1, HAL_MAX_DELAY);
	}

	ret=HAL_I2C_Master_Transmit_IT(&hi2c1, 0x3C, buf, 1);
	ret = HAL_I2C_IsDeviceReady(&hi2c1, 0x3C, 1, HAL_MAX_DELAY);
	while(ret !=HAL_OK)
	{
		HAL_I2C_Master_Abort_IT(&hi2c1, 0x3C);
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret = HAL_I2C_IsDeviceReady(&hi2c1, 0x3C, 1, HAL_MAX_DELAY);
	}


	ret=HAL_I2C_Master_Receive_IT(&hi2c1, 0x3C, buf, 6);
	while(ret !=HAL_OK)
	{
		HAL_I2C_Master_Abort_IT(&hi2c1, 0x3C);
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		ret=HAL_I2C_Master_Receive(&hi2c1, 0x3C, buf, 6,HAL_MAX_DELAY);
	}



				temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
				temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
				temp3 = ((int16_t)buf[4]<<8) | (buf[5]);

				mag.x = temp1/1.0f;

				mag.z = temp2/1.0f;

				mag.y = temp3/1.0f;


				mag.x -= ((xMax + xMin) / 2.0);
				mag.y -= ((yMax + yMin) / 2.0);
				mag.z -= ((zMax + zMin) / 2.0);
				f32  Roll  = - body->phib*DEG_TO_RAD;
				f32 Pitch = -body->thetab*DEG_TO_RAD;

			f32 Xh = mag.x * cos(Pitch) + mag.z * sin(Pitch);
			f32 Yh = mag.x * sin(Roll) * sin(Pitch) + mag.y * cos(Roll) + mag.z * sin(Roll) * cos(Pitch);



				body->psic= atan2(Xh,Yh) * RAD_TO_DEG;


				 body->psic += Declination;
					if (body->psic>180)									/* Due to declination check for >360 degree */
						body->psic = body->psic - 360;
						if (body->psic<-180)										/* Check for sign */
							body->psic = body->psic + 360;
}
