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
#define MAX 100
#define ARMED 595
#define F_max 54
#define F_min -54


void ARM_Motors(void);
void PWM(f32 dutyCycle, u8 motorNumber);
void DISARM_Motors(void);
void vCalibrate_Motors(void);
void vdFreeRunPWM(void);

#endif /* PWM_H_ */
