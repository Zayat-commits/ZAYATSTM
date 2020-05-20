

#include "compass.h"

f32 xMax, yMax, xMin, yMin,zMin, zMax;
f32     Mag_x_offset,    Mag_y_offset ,   Mag_z_offset ;
accel mag;
u8 buf[10];
int16_t temp1,temp2,temp3;

void Compass_Init()
{

	HAL_Delay(150);


	buf[0]=0x37; //Bypassing
	buf[1]=0x02;
	HAL_I2C_Master_Transmit(&hi2c1, 0b11010000, buf, 2, HAL_MAX_DELAY);



	xMax= yMax= xMin= yMin=zMax= zMin=0.0;


	buf[0]=0x00; //initialising the compass
	buf[1]=0x70;
	buf[2]=0xA0;
	buf[3]=0x00;

	HAL_I2C_Master_Transmit(&hi2c1, 0x3C, buf, 4, HAL_MAX_DELAY);


//
//
//		buf[0]=0x03;
//		HAL_I2C_Master_Transmit(&hi2c1, 0x3C, buf, 1, HAL_MAX_DELAY);
//		HAL_I2C_Master_Receive(&hi2c1, 0x3C, buf, 6, HAL_MAX_DELAY);
//
//					temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
//					temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
//					temp3 = ((int16_t)buf[4]<<8) | (buf[5]);
//
//					mag->x = temp1/1.0f;
//
//					mag->z = temp2/1.0f;
//
//					mag->y = temp3/1.0f;
//
//
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
//					mag->x = temp1/1.0f;
//
//					mag->z = temp2/1.0f;
//
//					mag->y = temp3/1.0f;
//
//				HAL_Delay(10);
//		}

	xMax=  -171; xMin=340; yMax= -383; yMin=50; zMax= -256;zMin=255;
	Mag_x_offset= (xMax+xMin)/2.0;
	Mag_y_offset= (yMax+yMin)/2.0;
	Mag_z_offset= (zMax+zMin)/2.0;

//send smth like DONE CALIB


}

void Read_Compass_Values(parameters *body)
{


	buf[0]=0x03;
	HAL_I2C_Master_Transmit(&hi2c1, 0x3C, buf, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, 0x3C, buf, 6, HAL_MAX_DELAY);

				temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
				temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
				temp3 = ((int16_t)buf[4]<<8) | (buf[5]);

				mag.x = temp1/1.0f;

				mag.z = temp2/1.0f;

				mag.y = temp3/1.0f;


	mag.x -= ((xMax + xMin) / 2.0);
	mag.y -= ((yMax + yMin) / 2.0);
	mag.z -= ((zMax + zMin) / 2.0);

	f32 euler[3]={body->phib, body->thetab,body->psib};
	f32 q[4];
	Quaternion(q, euler);
	f32 magz[3]={mag.x,mag.y,mag.z};
	Rotate_BtoW(magz, q);



	body->psic= atan2(magz[1],magz[0])*RAD_TO_DEG;











	//f32 Mag_roll= atan2(body->y_dot_dot,sqrt(body->x_dot_dot * body->x_dot_dot + body->z_dot_dot* body->z_dot_dot));
	//f32 Mag_pitch=-atan2(-body->x_dot_dot, sqrt(body->y_dot_dot * body->y_dot_dot + body->z_dot_dot * body->z_dot_dot));
	//f32 Mag_roll= body->phi;
	//f32 Mag_pitch= body->theta;


	//f32 Mag_x_hor = mag.y * sin(Mag_pitch)* sin(Mag_roll) +mag.x * cos(Mag_pitch) + mag.z * sin(Mag_pitch) * cos(Mag_roll);
	// f32 Mag_y_hor = mag.y * cos(Mag_roll) - mag.z * sin(Mag_roll);


	// body->psib = atan2( Mag_y_hor,Mag_x_hor) * RAD_TO_DEG;  // Magnetic North*/
	//body->psib = 180*atan2(-Mag_y_hor,Mag_x_hor)/M_PI;








	 body->psic += Declination;
		if (body->psic>180)									/* Due to declination check for >360 degree */
			body->psic = body->psic - 360;
			if (body->psic<-180)										/* Check for sign */
				body->psic = body->psic + 360;
}
