/*
 * STD_TYPES.h
 *
 *  Created on: 26 Aug 2017
 *      Author: Abdulrahman
 */

#ifndef STD_TYPES_H_
#define STD_TYPES_H_

typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned long int u32;

typedef signed char s8;
typedef signed short int s16;
typedef signed long int s32;

typedef float f32;
typedef double f64;

#define INTEGRAL_DT 0.025

/*All the parameters needed with a total size of 168 Bytes*/
typedef struct parameters{
	f32 x, y, z, z_baro;
	f32 x_dot, y_dot, z_dot;
	f32 x_dot_dot, y_dot_dot, z_dot_dot;
	f32 phi, theta, psi;
	f32 phib, thetab, psib ,psic;
	f32 phi_dot, theta_dot, psi_dot;
	f32 p, q, r;
	f32 p_dot, q_dot, r_dot;
	f32 x_cmd, y_cmd, z_cmd;
	f32 x_dot_cmd, y_dot_cmd, z_dot_cmd;
	f32 x_dot_dot_cmd, y_dot_dot_cmd, z_dot_dot_cmd;
	f32 psi_cmd, p_cmd, q_cmd, r_cmd;
	f32 u1, u2, u3, u4;
	f32 cmd_thrust[4];
	f32 xgps,ygps,zgps,vxgps,vygps,vzgps;
	struct status
	{
		u8 calibrated;
		u8 armed;
		u8 compass_state;
		u8 mpu_state;
		u8 gps_state;
		u8 ekf_state;
		u8 pwm;
		u8 busystate;
	}status;
	u8 ret_flag;
	f32 motor1,motor2,motor3,motor4;
} parameters;

typedef struct accel{
	f32 x, y, z;
}accel;

typedef struct bodyrate{
	f32 p, q, r;
}bodyrate;
#endif /* STD_TYPES_H_*/
