/*
 * estimation.c
 *
 *  Created on: Jun 5, 2020
 *      Author: Khaled Ali
 */
#include "main.h"
extern parameters parameter;
f32 ekfcov[7][7], hprimegps[6][7], hprimegpsT[7][6], hprimeMag[7] = {0},  R_GPS[6][6],Q_load[7][7], R_mag, dt = 0.15,inverse[6][6], toinvert[6][6], K[7][6], gprime[7][7] = {0};
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
	ekfcov[0][0] = 0.01;;
	ekfcov[1][1] = 0.01;;
	ekfcov[2][2] = 0.09;
	ekfcov[3][3] = 0.01;;
	ekfcov[4][4] = 0.01;;
	ekfcov[5][5] = 0.09;
	ekfcov[6][6] = 0.0025;
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
	Q_load[6][6] = 0.0064 * dt;
	R_mag = 0.01;
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
{ 		f32 dt = 0.1;

	  f32 cosTheta = cos(ptr->theta);
	  f32 sinTheta = sin(ptr->theta);

	  f32 cosPhi = cos(ptr->phi);
	  f32 sinPhi = sin(ptr->phi);

	  f32 sinPsi = sin(ptr->psi);
	  f32 cosPsi = cos(ptr->psi);
	f32 Rprime00 =  - cosTheta * sinPsi;
	f32 Rprime01 =- sinPhi  * sinTheta * sinPsi - cosTheta * cosPsi;
	f32 Rprime02 = - cosPhi  * sinTheta * sinPsi + sinPhi   * cosPsi;
	f32 Rprime10 = cosTheta * cosPsi;
	f32 Rprime11 = sinPhi  * sinTheta * cosPsi - cosPhi * sinPsi;
	f32 Rprime12 = cosPhi  * sinTheta * cosPsi + sinPhi * sinPsi;

	gprime[0][3] = dt;
	gprime[1][4] = dt;
	gprime[2][5] = dt;
	gprime[3][6] = (Rprime00 * ptr->x_dot_dot + Rprime01 * ptr->y_dot_dot + Rprime02 * ptr->z_dot_dot) * dt;
	gprime[4][6] = (Rprime10 * ptr->x_dot_dot + Rprime11 * ptr->y_dot_dot + Rprime12 * ptr->z_dot_dot) * dt;
	gprime[5][6] = 0;
	for (int i = 0; i<7; i++)
		gprime[i][i] = 1;
	f32 gprimeT[7][7];
	for (int i=0;i<7;i++)
		for(int j=0; j<7; j++)
			gprimeT[i][j]=gprime[j][i];
	f32 ekfcovnew1[7][7];
	f32 ekfcovnew2[7][7];
	matrix_multi(7,7,7, gprime,ekfcov,ekfcovnew1);
	matrix_multi(7,7,7, ekfcovnew1,gprimeT,ekfcovnew2);
	for (int i=0;i<7;i++)
		for (int j=0;j<7;j++)
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
		{	hprimegps[i][i] = 1;
			hprimegpsT[i][i] = 1;
		}
		f32 sum=0;
		f32 temp1[6][7];
		f32 temp2[6][6];
		f32 temp3[7][6];
		f32 temp4[7][1], temp5[7][7], temp6[7][7];
		matrix_multi(6,7,7, hprimegps,ekfcov,temp1);
		matrix_multi(6,6,7, temp1,hprimegpsT,temp2);

		for (int i=0;i<6;i++)
			for (int j=0;j<6;j++)
				toinvert[i][j] = temp2[i][j] + R_GPS[i][j];
		cofactor(toinvert, 6);
		matrix_multi(7,6,7, ekfcov,hprimegpsT,temp3);
		for (int i=0;i<7;i++){
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
		matrix_multi(7,1,6, K,z,temp4);
		ptr->x += temp4[0][0];
		ptr->y += temp4[1][0];
//		ptr->z += temp4[2][0]; ////////////////////////////////////// EXCLUDE Z
		ptr->x_dot += temp4[3][0];
		ptr->y_dot += temp4[4][0];
		ptr->z_dot += temp4[5][0];
		matrix_multi(7,7,6, K,hprimegps, temp5);
		for(int i = 0; i<7; i++){
			for(int j = 0; j<7;j++)
			{
				if(i == j)
					temp5[i][j] = 1 - temp5[i][j];
				else
					temp5[i][j] *= -1;
			}
		}
	matrix_multi(7,7,7, temp5,ekfcov, temp6);
		for (int i=0;i<7;i++)
			for (int j=0; j<7;j++)
				ekfcov[i][j] = temp6[i][j];
	}
}
void updatefromMag(parameters *ptr)
{
	f32 z_mag, zfromX_mag = ptr -> psi;
	f32 diff = z_mag - zfromX_mag;
	if (diff > 180)
		zfromX_mag += 360;
	else if ( diff < -180)
		zfromX_mag -= 360;
	hprimeMag[6] = 1;
	f32 temp1[7], temp2, temp3[7][7], temp4[7][7];
	f32 sum=0;
		for (int j=0; j<7;j++){
			for (int k=0; k<7;k++){
				sum = sum + hprimeMag[k]*ekfcov[k][j];
			}
			temp1[j] = sum;
			sum = 0;
		}

		for (int i =0; i<7; i++)
			sum += temp1[i] * hprimeMag[i];
		temp2 = sum;
		sum = 0;
		temp2 += R_mag;
		temp2 = 1/temp2;
		for (int j=0; j<7;j++){
			for (int k=0; k<7;k++){
				sum = sum + ekfcov[j][k] * hprimeMag[k];
			}
			temp1[j] = sum * temp2;
			sum = 0;
		}
		ptr->psi = ptr->psi + temp1[6] * (zfromX_mag - z_mag);
		for (int i=0;i<7;i++){
			for (int j=0; j<7;j++){
					if (i == j)
						temp3[i][j] = 1 - temp1[i] * hprimeMag[j];
					else
						temp3[i][j] = -1 * temp1[i] * hprimeMag[j];
			}
		}
		matrix_multi(7,7,7, temp3,ekfcov, temp4);
		for (int i=0;i<7;i++)
			for (int j=0;j<7;j++)
				ekfcov[i][j] = temp4[i][j];

}

