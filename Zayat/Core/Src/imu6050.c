 /*
 * imu6050.c
 *
 *  Created on: Oct 3, 2019
 *      Author: Khaled Ali
 */
/*
 * IMU_REGS.c
 *
 *  Created on: Oct 3, 2019
 *      Author: Zayat
 */
#include "main.h"
extern I2C_HandleTypeDef hi2c1;

/********************************************************************************************************************************************************
 * **************************************************THIS CODE WAS WRITTEN BY TEAM RAVENS****************************************************************
 *
 * This section of code will demonstrate how to communicate with MPU6050.
 *
 * Process:
 * 			1- Initialize I2C protocol.
 * 			2- Initialize MPU by setting config, sample rate, power management and start location adderss.
 * 			3- MPU slave address is = 0x68, thus while communicating with it we use it but shifted to the left once -> 0xD0,
 * 				and the last bit defines either read or write (0xD0 for write, 0xD1 for read)
 * 			4- Set INT enabled to mark end of config.
 * 			5- Read Register raw values (ACCEL_X -> ACCEL_Y -> ACCEL_Z -> TEMP - > GYRO_X -> GYRO_Y -> GYRO_Z) each is 2 parts: HIGH then LOW.
 * 			6- Divide each register by its sensitivity for its actual values.
 *
 * *******************************************************************************************************************************************************
 */
	f32 errorp =0 ; // used for calibration
	f32 errorq =0 ;
	f32 errorr =0 ;
	f32 errorx =0 ; // used for calibration
	f32 errory =0 ;
	f32 errorz =0 ;
	f32 errorxbody =0 ; // used for calibration
	f32 errorybody =0 ;
	f32 errorzbody =0 ;

	u8 buf[10];

void MPU_Init(parameters *p, const f32 RT)
{
	HAL_Delay(150);
	buf[0]=SMPRLT_DIV;
	buf[1]=0x00;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 2, HAL_MAX_DELAY);

	buf[0]=PWR_MGMT_1;
	buf[1]=0x01;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 2, HAL_MAX_DELAY);


	buf[0]=GYRO_CONFIG;
	buf[1]=0x18;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 2, HAL_MAX_DELAY);		//Set FSEL to 3 -> range = -2000 to 2000 degrees per sec.



	buf[0]=ACCEL_CONFIG;
	buf[1]=0x18;//Set AFS_SEL to 3 -> range = -16g to 16g
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 2, HAL_MAX_DELAY);

	buf[0]=INT_ENABLE;
	buf[1]=0x01;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 2, HAL_MAX_DELAY);


	// Calibrating IMU



	buf[0]=GYRO_XOUT_H;
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 1, HAL_MAX_DELAY);

	HAL_I2C_Master_Receive(&hi2c1, 0xD0, buf, 6, HAL_MAX_DELAY);



	int16_t temp1,temp2,temp3;
				temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
				temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
				temp3 = ((int16_t)buf[4]<<8) | (buf[5]);

					p->p = temp1/1.0f;

					p->q = temp2/1.0f;

					p->r = temp3/1.0f;


	for (u16 i=0 ; i<1000 ; i++)
	{
		errorp= errorp + p->p;
		errorq= errorq + p->q;
		errorr= errorr + p->r;

		buf[0]=GYRO_XOUT_H;
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 1, HAL_MAX_DELAY);
			HAL_I2C_Master_Receive(&hi2c1, 0xD0, buf, 6, HAL_MAX_DELAY);
			int16_t temp1,temp2,temp3;
			temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
			temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
			temp3 = ((int16_t)buf[4]<<8) | (buf[5]);

				p->p = temp1/1.0f;

				p->q = temp2/1.0f;

				p->r = temp3/1.0f;


				buf[0]=ACCEL_XOUT_H;
					HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 1, HAL_MAX_DELAY);
					HAL_I2C_Master_Receive(&hi2c1, 0xD0, buf, 6, HAL_MAX_DELAY);

					temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
					temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
					temp3 = ((int16_t)buf[4]<<8) | (buf[5]);


				errorxbody= errorxbody + temp1/1.0f;
				errorybody= errorybody + temp2/1.0f;
				errorzbody= errorzbody + temp3/1.0f-2048;





			HAL_Delay(3);
	}
	errorp=errorp/1000.0;
	errorq=errorq/1000.0;
	errorr=errorr/1000.0;
	errorxbody= errorxbody /1000.0;
	errorybody= errorybody /1000.0;
	errorzbody= errorzbody /1000.0;


}
void Accel_calibration(parameters *p, f32 RT)
{
		f32 q[4], euler[3];
		Read_Gyro_Values(p, RT);
		Read_Accel_Values(p);
		euler[0]= p->phib;
		euler[1]= p->thetab;
		euler[2]= p->psib;
		Quaternion(q, euler);
		f32 acc[3] = {p->x_dot_dot, p->y_dot_dot, p->z_dot_dot};

		Rotate_BtoW_acc(acc, q);
		for (u16 i=0 ; i<300 ; i++)
		{
			errorx= errorx + acc[0];
			errory= errory + acc[1];
			errorz= errorz + acc[2];




			Read_Gyro_Values(p, RT);
			Read_Accel_Values(p);
			euler[0] = p->phib; euler[1] = p->thetab; euler[2] = p->psib;
			Quaternion(q, euler);
			acc[0] = p->x_dot_dot; acc[1] = p->y_dot_dot; acc[2] = p->z_dot_dot;
			Rotate_BtoW_acc(acc, q);
				HAL_Delay(3);
		}
		errorx=errorx/300.0;
		errory=errory/300.0;
		errorz=errorz/300.0;

}
void Read_Accel_Values(parameters *p)
{

	buf[0]=ACCEL_XOUT_H;
		  HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 1, HAL_MAX_DELAY);
		 HAL_I2C_Master_Receive(&hi2c1, 0xD0, buf, 6, HAL_MAX_DELAY);
		int16_t temp1,temp2,temp3;
		temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
		temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
		temp3 = ((int16_t)buf[4]<<8) | (buf[5]);
			p->x_dot_dot = (temp1-errorxbody)/2048.0;
			p->y_dot_dot = (temp2-errorybody)/2048.0;
			p->z_dot_dot = (temp3-errorzbody)/2048.0;
//			p->x_dot_dot = (temp1)/2048.0f;
//			p->y_dot_dot = (temp2)/2048.0f;
//			p->z_dot_dot = (temp3)/2048.0f;

}

void Read_Gyro_Values(parameters *p, const f32 RT)
{

		buf[0]=GYRO_XOUT_H;
		HAL_I2C_Master_Transmit(&hi2c1, 0xD0, buf, 1, HAL_MAX_DELAY);

		HAL_I2C_Master_Receive(&hi2c1, 0xD0, buf, 6, HAL_MAX_DELAY);


		int16_t temp1,temp2,temp3;
					temp1 = ((int16_t)buf[0]<<8) | (buf[1]);
					temp2 = ((int16_t)buf[2]<<8) | (buf[3]);
					temp3 = ((int16_t)buf[4]<<8) | (buf[5]);

						p->p = temp1/1.0f;

						p->q = temp2/1.0f;

						p->r = temp3/1.0f;
		p->p = ((p->p-errorp)/16.4);
		p->q = ((p->q-errorq)/16.4);
		p->r = ((p->r-errorr)/16.4);
	//Body frame phi and theta
	p->phib = p->phib+RT*p->p ;
	p->thetab = p->thetab+RT*p->q;
	p->psib = p->psib-RT*p->r;

	f32 first_term;
	f32 second_term;
	first_term = 0.98;
	second_term = 0.02;
	f32 accelPhi;
	f32 acceltheta;


    accelPhi = atan2(p->y_dot_dot,sqrt(p->x_dot_dot * p->x_dot_dot + p->z_dot_dot* p->z_dot_dot)) * 57.3;
   acceltheta = atan2(-p->x_dot_dot, sqrt(p->y_dot_dot * p->y_dot_dot + p->z_dot_dot * p->z_dot_dot)) * 57.3;


   p->phib = first_term * p->phib + second_term * accelPhi;
   p->thetab = first_term * p->thetab + second_term * acceltheta;
   if (p->psib*p->psic <0)
	   p->psib=p->psic;
   else
   p->psib = first_term * p->psib + second_term * p->psic;

	if(p->psib > 180.0)
		p->psib -= 360;
	if(p->psib < -180.0)
		p->psib += 360;

}

void imu_Comp_Filter(parameters *p, const f32 RT)
{

	f32 q[4], euler[3], ang_vel[3];

	f32 first_term;
	f32 second_term;
	f32 accelPhi;
	f32 acceltheta;
	//euler[0] = p->phi; euler[1] = p->theta; euler[2] = p->psi;
	euler[0]= p->phib;
	euler[1]= p->thetab;
	euler[2]= p->psib;

	Quaternion(q, euler);
	ang_vel[0] = p->p; ang_vel[1] = p->q; ang_vel[2] = p->r;
	f32 acc_vel[3];
	acc_vel[0]= p->x_dot_dot;
	acc_vel[1]= p->y_dot_dot;
	acc_vel[2]= p->z_dot_dot;
	Rotate_BtoW(ang_vel,q);
//----------------------------------------------------------------------------

	f32 acc[3] = {p->x_dot_dot, p->y_dot_dot, p->z_dot_dot};
	Rotate_BtoW(acc, q); // there was a line to get q but i believe it's already there



	p->x += RT * p->x_dot;
	p->y += RT * p->y_dot;
	p->z += RT * p->z_dot;
//	p->z = 0.99*p->z +0.01*p->z_baro; ///////////////////////////////////////////////////


	p->x_dot += RT * (acc[0] ) * 9.8;
	p->y_dot += RT * (acc[1] ) * 9.8;
	p->z_dot += RT * (acc[2]-1 ) * 9.8;


	// Integrating the p q r to get angles
	p->phi = p->phi +ang_vel[0]*RT;
	p->theta =p->theta+ ang_vel[1]*RT;
	p->psi=p->psib;
	//p->psi =p->psi+ ang_vel[2]*RT;

	// Accelometer from body to world
	f32 temp1 = cos(-p->psi*0.0174532)*p->x_dot_dot + sin(-p->psi*0.0174532)*p->y_dot_dot;
	f32 temp2 = -sin(-p->psi*0.0174532)*p->x_dot_dot + cos(-p->psi*0.0174532)*p->y_dot_dot;
	acc_vel[0]=temp1;
	acc_vel[1]=temp2;


	// Angles from Acc
    accelPhi = atan2(acc_vel[1],sqrt(acc_vel[0] * acc_vel[0] + acc_vel[2]* acc_vel[2])) * 57.3;
   acceltheta = atan2(-acc_vel[0], sqrt(acc_vel[1] * acc_vel[1] + acc_vel[2] * acc_vel[2])) * 57.3;

	 first_term = 0.98; ////////////////////
	 second_term = 0.02; //////////////////

   // Comp Filter
    p->phi = first_term * p->phi + second_term * accelPhi;
    p->theta = first_term * p->theta + second_term * acceltheta;
	if(p->psi > 180.0)
		p->psi -= 360;
	if(p->psi < -180.0)
		p->psi += 360;


	p->x_dot_dot = acc[0];
	p->y_dot_dot = acc[1];
	p->z_dot_dot = acc[2];


	//predict(p);
}

