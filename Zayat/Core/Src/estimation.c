/*
 * estimation.c
 *
 *  Created on: Jun 5, 2020
 *      Author: Khaled Ali
 */
#include "main.h"
extern parameters parameter;
f32 ekfcov[6][6], hprimegps[6][6], hprimegpsT[6][6],  R_GPS[6][6],Q_load[6][6], dt = 0.15,inverse[6][6], toinvert[6][6], K[6][6], gprime[6][6] = {0};
f32 z[6][1], zfromX[6];
accel distance;
accel speed;
void matrix_multi(int o, int n, int l,f32 mat1[][l],f32 mat2[][n], f32 mat3[][n])
{	f32 sum =0;
	for (int i=0;i<o;i++){
		for (int j=0; j<n;j++){
			for (int k=0; k<l;k++){
				sum = sum + mat1[i][k] * mat2[k][j];
			}
			mat3[i][j] = sum;
			sum = 0;
		}
	}
}
void init_EKF(void)
{

	R_GPS[0][0] = 1;
	R_GPS[1][1] = 1;
	R_GPS[2][2] = 1; //we'll see
	R_GPS[3][3] = 0.01;
	R_GPS[4][4] = 0.01;
	R_GPS[5][5] = 0.09;
	Q_load[0][0] = 0.0025 * dt;
	Q_load[1][1] = 0.0025 * dt;
	Q_load[2][2] = 0.0025 * dt;
	Q_load[3][3] = 0.04 * dt;
	Q_load[4][4] = 0.04 * dt;
	Q_load[5][5] = 0.01 * dt;
	parameter.status.ekf_state = 1;
}
void predictstate(parameters *ptr)
{
	zfromX[0] = ptr->x;
	zfromX[1] = ptr->y;
	zfromX[2] = ptr->z;
	zfromX[3] = ptr->x_dot;
	zfromX[4] = ptr->y_dot;
	zfromX[5] =	ptr->z_dot;
}
void predict(parameters *ptr)
{
	gprime[0][3] = dt;
	gprime[1][4] = dt;
	gprime[2][5] = dt;
	for (int i = 0; i<6; i++)
		gprime[i][i] = 1;
	f32 gprimeT[6][6];
	for (int i=0;i<6;i++)
		for(int j=0; j<6; j++)
			gprimeT[i][j]=gprime[j][i];
	f32 ekfcovnew1[6][6];
	f32 ekfcovnew2[6][6];
	matrix_multi(6,6,6, gprime,ekfcov,ekfcovnew1);
	matrix_multi(6,6,6, ekfcovnew1,gprimeT,ekfcovnew2);
	for (int i=0;i<6;i++)
		for (int j=0;j<6;j++)
			ekfcov[i][j] = ekfcovnew2[i][j] + Q_load[i][j];
}
void updatefromGps(parameters *ptr)
{
	predictstate(ptr);
	predict(ptr);
	u8 fix;
	accel *position = &distance;
	accel *velocity = &speed;
	fix = Read_gps(position, velocity);
	if (fix==3)
	{
		z[0][0] = position-> x;
		z[1][0] = position-> y;
		z[2][0] = position-> z;
		z[3][0] = velocity-> x;
		z[4][0] = velocity-> y;
		z[5][0] = velocity-> z;
		for (int i =0; i<6; i++)
			hprimegps[i][i] = 1;
		for (int i=0;i<6;i++)
			for (int j=0;j<6;j++)
				hprimegpsT[j][i] = hprimegps[i][j];
		f32 sum=0;
		f32 temp1[6][6];
		f32 temp2[6][6];
		f32 temp3[6][6];
		f32 temp4[6][1], temp5[6][6], temp6[6][6];
		matrix_multi(6,6,6, hprimegps,ekfcov,temp1);
		matrix_multi(6,6,6, temp1,hprimegpsT,temp2);

		for (int i=0;i<6;i++)
			for (int j=0;j<6;j++)
				toinvert[i][j] = temp2[i][j] + R_GPS[i][j];
		int r = cofactor(toinvert, 6);
		if (r != 1)
		{matrix_multi(6,6,6, ekfcov,hprimegpsT,temp3);
		for (int i=0;i<6;i++){
			for (int j=0; j<6;j++){
				for (int k=0; k<6;k++){
					sum = sum + temp3[i][k] * inverse[k][j];
				}
				K[i][j] = sum;
				sum = 0;
			}
		}

		for(int k=0; k<6; k++)
			z[k][0] = z[k][0] - zfromX[k];
		matrix_multi(6,1,6, K,z,temp4);
		ptr->x += temp4[0][0];
		ptr->y += temp4[1][0];
//		ptr->z += temp4[2][0]; ////////////////////////////////////// EXCLUDE Z
		ptr->x_dot += temp4[3][0];
		ptr->y_dot += temp4[4][0];
		ptr->z_dot += temp4[5][0];
		matrix_multi(6,6,6, K,hprimegps, temp5);
		for(int i = 0; i<6; i++){
			for(int j = 0; j<6;j++)
			{
				if(i == j)
					temp5[i][j] = 1 - temp5[i][j];
				else
					temp5[i][j] *= -1;
			}
		}
	matrix_multi(6,6,6, temp5,ekfcov, temp6);
		for (int i=0;i<6;i++)
			for (int j=0; j<6;j++)
				ekfcov[i][j] = temp6[i][j];
	}
	}
}

