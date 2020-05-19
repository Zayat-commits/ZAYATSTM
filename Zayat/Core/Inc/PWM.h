/*
 * PWM.h
 *
 *  Created on: Feb 10, 2020
 *      Author: Yousef
 */

#ifndef PWM_H_
#define PWM_H_


#include "STD_TYPES.h"
#include "main.h"
#define MIN 0
#define MAX 100
#define ARMED 595
#define F_max 600
#define F_min -3


void ARM_Motors(void);
void PWM(u16 dutyCycle, u8 motorNumber);
void DISARM_Motors(void);
void vCalibrate_Motors(void);

#endif /* PWM_H_ */
