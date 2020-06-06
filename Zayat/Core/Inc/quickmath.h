/*
 * quickmath.h
 *
 *  Created on: Feb 18, 2020
 *      Author: Khaled Ali
 */

#ifndef QUICKMATH_H_
#define QUICKMATH_H_

#define PI 3.14159
#define TERMS 7
float power(float base, int exp);
int fact(int n);
float determinant(float a[6][6], int k);
void cofactor(float num[6][6], int f);
void transpose(float num[6][6], float fac[6][6], int r);
#endif /* QUICKMATH_H_ */
