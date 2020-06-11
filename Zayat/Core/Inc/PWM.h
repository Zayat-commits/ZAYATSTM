/*
 * PWM.h
 *
 *  Created on: Feb 10, 2020
 *      Author: Yousef
 */

#ifndef PWM_H_
#define PWM_H_


#include "main.h"
#define MIN 0
#define MAX 1200
#define F_max 792
#define HOVER 480
#define LANDING 360
#define F_min 168


void ARM_Motors(void);
void PWM(u32 dutyCycle, u8 motorNumber);
void DISARM_Motors(void);
void vCalibrate_Motors(void);
void vdFreeRunPWM(void);

#endif /* PWM_H_ */
