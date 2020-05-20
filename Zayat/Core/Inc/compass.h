/*
 * compass.h
 *
 *  Created on: Feb 23, 2020
 *      Author: mahmo
 */

#ifndef COMPASS_H_
#define COMPASS_H_
#define PI 3.14159
#define  DEG_TO_RAD 0.01745
#define  RAD_TO_DEG 57.295

#include "main.h"
#include "math.h"
#include "quickmath.h"
#include "stdlib.h"
#include "STD_TYPES.h"
#include "IMU_REGS.h"
extern I2C_HandleTypeDef hi2c1;



#define Declination 4.526366582

void Compass_Init();

void Read_Compass_Values(parameters *body);




#endif /* COMPASS_H_ */
