/*
 * IMU_REGS.h
 *
 *  Created on: Oct 3, 2019
 *      Author: Zayat
 */

#ifndef IMU_REGS_H_
#define IMU_REGS_H_

/* *****************************************************************************************************************************
 *
 * This header file contains all the addresses to the registers inside the IMU such as output REGS, CONFIG REGS and SAMPLE RATE
 *
 * *****************************************************************************************************************************/

#define SMPRLT_DIV 0x19						//Sets sample rate through equation: Gyro Output Rate/(1 + SMPLRT_DIV)
#define CONFIG_REG 0x1A						//Configures bandwidth for Gyro & Acce, see table at page 13
#define GYRO_CONFIG 0x1B					//Sets Gyro range through FS_SEL (0-3) -> (250/s - 2000/s)
#define ACCEL_CONFIG 0x1C					//Sets Accel range though AFS_SEL (0-3) ->(2g - 16g)
#define INT_ENABLE 0x38						//Set the LSB to 1 when data write to all registers are done (After configuring in the main)
#define ACCEL_XOUT_H 0x3B					/////////////////////////////////////////////////////////////////////////////////////
#define ACCEL_XOUT_L 0x3C					//
#define ACCEL_YOUT_H 0x3D					//			All outputs of the ACCEL, each in 16 bits separated into 2 REGS
#define ACCEL_YOUT_L 0x3E					//			The ACCEL sends the HIGH REG first then the LOW REG
#define ACCEL_ZOUT_H 0x3F					//
#define ACCEL_ZOUT_L 0x40					/////////////////////////////////////////////////////////////////////////////////////
#define TEMP_OUT_H 0x41						// Temp sensor output in C = TEMP_OUT / 340 + 36.53
#define TEMP_OUT_L 0x42						//TEMP_OUT = [TEMP_OUT_H ][TEMP_OUT_L]
#define GYRO_XOUT_H 0x43					/////////////////////////////////////////////////////////////////////////////////////
#define GYRO_XOUT_L 0x44					//
#define GYRO_YOUT_H 0x45					//			All outputs of the GYRO, each in 16 bits separated into 2 REGS
#define GYRO_YOUT_L 0x46					//			The GYRO sends the HIGH REG first then the LOW REG
#define GYRO_ZOUT_H 0x47					//
#define GYRO_ZOUT_L 0x48					/////////////////////////////////////////////////////////////////////////////////////
#define PWR_MGMT_1 0x6B						// Decides the clock (by default 8 MHz) but should be set to a reference (i.e. X-axes Gyro reference), set CLKSEL to 010

#endif /* IMU_REGS_H_ */
