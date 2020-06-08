
#ifndef IMU6050_H_
#define IMU6050_H_
#include "IMU_REGS.h"
#include "STD_TYPES.h"
#include "main.h"
#include "quickmath.h"
void MPU_Init(parameters *p, const f32 RT);
void Read_Accel_Values(parameters *p);
void Read_Gyro_Values(parameters *p, const f32 RT);
void imu_Comp_Filter(parameters *p, const f32 RT);
#endif /* IMU6050_H_ */
