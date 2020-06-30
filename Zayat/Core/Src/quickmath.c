/*
 * quickmath.c
 *
 *  Created on: Feb 18, 2020
 *      Author: Khaled Ali
 */
#include "main.h"
extern f32 toinvert[6][12];
f32 v = 0;
float power(float base, int exp) {
    if(exp < 0) {
        if(base == 0)
            return -0; // Error!!
        return 1 / (base * power(base, (-exp) - 1));
    }
    if(exp == 0)
        return 1;
    if(exp == 1)
        return base;
    return base * power(base, exp - 1);
}

int fact(int n) {
    return n <= 0 ? 1 : n * fact(n-1);
}

void Quaternion(f32 *q, f32 *euler)
{


	euler[0]/=2;
	euler[1]/=2;
	euler[2]/=2;
	euler[0]*=0.0174532;
	euler[1]*=0.0174532;
	euler[2]*=0.0174532;
	q[0] = cos(euler[0]) * cos(euler[1]) * cos(euler[2])   + sin(euler[0])  * sin(euler[1]) * sin(euler[2]);
	q[1] =-cos(euler[0]) * sin(euler[1]) * sin(euler[2])   + cos(euler[1]) * cos(euler[2])   * sin(euler[0]);
	q[2] = cos(euler[0]) * cos(euler[2])   * sin(euler[1]) + sin(euler[0])  * cos(euler[1]) * sin(euler[2]);
	q[3] = cos(euler[0]) * cos(euler[1]) * sin(euler[2])   - sin(euler[0])  * cos(euler[2])   * sin(euler[1]);

}
void Rotate_BtoW(f32 *acc, f32 *q)
{
	f32 R[9];
	f32 x = q[1];
	f32 y = q[2];
	f32 z = q[3];
	f32 s = q[0];



	 R[0] = 1 - 2*y*y - 2*z*z; R[1] = 2*x*y - 2*s*z;     R[2] = 2*x*z + 2*s*y;
	  R[3] = 2*x*y + 2*s*z;     R[4] = 1 - 2*x*x - 2*z*z; R[5] = 2*y*z - 2*s*x;
	  R[6] = 2*x*z - 2*s*y;     R[7] = 2*y*z + 2*s*x;     R[8] = 1 - 2*x*x - 2*y*y;





		f32 acc0 = R[0]*acc[0] + R[1]*acc[1] + R[2]*acc[2];
		f32 acc1 = R[3]*acc[0] + R[4]*acc[1] + R[5]*acc[2];
		f32 acc2 = R[6]*acc[0] + R[7]*acc[1] + R[8]*acc[2];
		acc[0]=acc0;
		acc[1]=acc1;
		acc[2]=acc2;

}
void Rotate_BtoW_acc(f32 *acc, f32 *q)
{
	f32 q0 = -q[0];
	f32 R[9];
	f32 acc2[3];
	f32 r0 = q0*q0;
	f32 r1 = q[1]*q[1];
	f32 r2 = q[2]*q[2];
	f32 r3 = q[3]*q[3];
	R[0] = r0 + r1 - r2 - r3;
	R[1] = 2*q[1]*q[2] + 2*q0*q[3];
	R[2] = 2*q[1]*q[3] - 2*q0*q[2];
	R[3] = 2*q[1]*q[2] - 2*q0*q[3];
	R[4] = r0 - r1 + r2 - r3;
	R[5] = 2*q[2]*q[3] + 2*q0*q[1];
	R[6] = 2*q[1]*q[3] + 2*q0*q[2];
	R[7] = 2*q[2]*q[3] - 2*q0*q[1];
	R[8] = r0 - r1 - r2 + r3;
	acc2[0] = R[0]*acc[0] + R[1]*acc[1] + R[2]*acc[2];
	acc2[1] = R[3]*acc[0] + R[4]*acc[1] + R[5]*acc[2];
	acc2[2] = R[6]*acc[0] + R[7]*acc[1] + R[8]*acc[2];
	acc[0] = acc2[0];
	acc[1] = acc2[1];
	acc[2] = acc2[2];

}
void matrix_inverse(int order)
{
	int qr,qc;
for (qr =0;qr<order;qr++)
{
	for(qc=order; qc<2*order;qc++)
	{
		toinvert[qr][qc]=0;
	}
}
    float temp;

    for (int i = 0; i < order; i++) {

        for (int j = 0; j < 2 * order; j++) {

            // Add '1' at the diagonal places of
            // the matrix to create a identity matirx
            if (j == (i + order))
                toinvert[i][j] = 1;
        }
    }

    // Interchange the row of matrix,
    // interchanging of row will start from the last row
  /*  for (int i = order - 1; i > 0; i--) {
         if (toinvert[i - 1][0] < toinvert[i][0])
         for (int j = 0; j < 2 * order; j++) {

        	 temp = toinvert[i][j];
        	 toinvert[i][j] = toinvert[i - 1][j];
        	 toinvert[i - 1][j] = temp;
            }


    }*/

    for (int i = 0; i < order; i++) {

        for (int j = 0; j < order; j++) {

        	if (toinvert[i][i]==1) {

        		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,1);

        		break;}

            if (j != i) {

                temp = toinvert[j][i] / toinvert[i][i];
                for (int k = 0; k < 2 * order; k++) {

                    toinvert[j][k] -= toinvert[i][k] * temp;
                }
            }
        }
    }

    // Multiply each row by a nonzero integer.
    // Divide row element by the diagonal element
    for (int i = 0; i < order; i++) {

        temp = toinvert[i][i];
        for (int j = 0; j < 2 * order; j++) {

            toinvert[i][j] = toinvert[i][j] / temp;
        }
    }
}

