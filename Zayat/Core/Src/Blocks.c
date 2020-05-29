/*
 * Blocks.c
 *
 *  Created on: May 20, 2020
 *      Author: abdul
 */

#include "Blocks.h"

void vdBodyRatesBlock(parameters* ptr)
{
	f32 p_error,q_error,r_error;
	p_error = ptr->p_cmd - ptr->p;
	q_error = ptr->q_cmd - ptr->q;
	r_error = ptr->r_cmd - ptr->r;
	ptr->p_dot = kp_p * p_error;
	ptr->q_dot = kp_q * q_error;
	ptr->r_dot = kp_r * r_error;
	ptr->u2 = Ixx * ptr->p_dot;
	ptr->u3 = Iyy * ptr->q_dot;
	ptr->u4 = Izz * ptr->r_dot;
}

void vdDroneStartBlock(parameters* ptr)
{
	u8 buffer[25];

	fview(PRINT_NORMAL, 0, "Insert Psi Commanded: ");
	string_receive((char*)buffer);
	ptr->psi_cmd = atoi(buffer)/1.0f;

  /*
switch (atoi(buffer))
	{
case 0:
		vCalibrate_Motors();
		break;
case 1:	ARM_Motors();
		PWM(ARMED,1);
		PWM(ARMED,2);
		PWM(ARMED,3);
		PWM(ARMED,4);
		break;
default: vCalibrate_Motors();
	}
vTaskDelete(NULL);
*/
}
void vdMPUBlock(parameters* ptr)
{
	static uint8_t adad=0;
	/*Read Gyro and Accel values, then comp filter*/
	Read_Accel_Values(ptr);
	Read_Gyro_Values(ptr,INTEGRAL_DT);
//	imu_Comp_Filter(ptr,INTEGRAL_DT);
//	Read_Compass_Values(ptr);

	/*Template of function fview()-> fview(PRINT_TYPE, VARIABLE, STATEMENT)*/
	if (adad ==10)
	{
		fview(PRINT_FLOAT_WITH_TAB, ptr->thetab, "Value of phi = ");
		fview(PRINT_FLOAT_WITH_TAB, ptr->phib, "Value of theta = ");
		fview(PRINT_FLOAT_NO_TAB, ptr->psic, "Value of psic = ");
		adad=0;
	}
	adad++;

}

void vdPrintBlock(parameters* ptr)
{

	/*---------------------------------*/
	/*TO READ FORCE VALS IN WORLD FRAME*/
	/*---------------------------------*/

	fview(PRINT_FLOAT_WITH_TAB, ptr->cmd_thrust[0], "Value of F1 = ");
	fview(PRINT_FLOAT_WITH_TAB, ptr->cmd_thrust[1], "Value of F2 = ");
	fview(PRINT_FLOAT_WITH_TAB, ptr->cmd_thrust[2], "Value of F3 = ");
	fview(PRINT_FLOAT_NO_TAB, ptr->cmd_thrust[3], "Value of F4 = ");
}

void vdInsertBlock(parameters* ptr)
{
	uint8_t buffer[25];
	uint8_t ok[] = {"OK"};
}

void vdOutputBlock(parameters* ptr)
{
	f32 l = L/1.4142135623;
	f32 t1 = ptr->u2/ l;
	f32 t2 = ptr->u3 / l;
	f32 t3 = - ptr->u4/ k_thrust;
	f32 t4 = ptr->u1;
	ptr->cmd_thrust[0] = (t1 + t2 + t3 + t4)/4.f; // front left
	ptr->cmd_thrust[1] = (-t1 + t2 - t3 + t4)/4.f; // front right
	ptr->cmd_thrust[2] = (t1 - t2 - t3 + t4)/4.f ; // rear left
	ptr->cmd_thrust[3] = (-t1 - t2 + t3 + t4)/4.f; // rear right

	/*-----------*/
	/*PWM MAPPING*/
	/*-----------*/
/*	u8 i;
	u16 speed_pwm[4];
	for(i=0 ; i<4;i++)
	{
		if (ptr->cmd_thrust[i] < F_min) ptr->cmd_thrust[i]=F_min;
		if (ptr->cmd_thrust[i] > F_max) ptr->cmd_thrust[i]=F_max;
		speed_pwm[i] = (1/(F_max-F_min))*ptr->cmd_thrust[i]*500.0;
		PWM(speed_pwm[i],i+1);
	}
*/
}

void vdRollPitchBlock(parameters* ptr)
{
	f32 b_x_dot_cmd, b_y_dot_cmd, taw=1/kp_bank;
	f32 R11 = 1;
	f32 R12 = sin(ptr->phi) * sin(ptr->theta) / cos(ptr->theta);
	f32 R13= cos(ptr->phi) * sin(ptr->theta) / cos(ptr->theta);
	f32 R21 = 0;
	f32 R22 = cos(ptr->phi);
	f32 R23= -sin(ptr->phi);
	f32 R31 = 0;
	f32 R32 = sin(ptr->phi) / cos(ptr->theta);
	f32 R33 = cos(ptr->phi) / cos(ptr->theta);
	f32 R13_cmd= ptr->x_dot_dot_cmd*m/ptr->u1;
	f32 R23_cmd= ptr->y_dot_dot_cmd*m/ptr->u1;
	b_x_dot_cmd= (R13-R13_cmd)/taw;
	b_y_dot_cmd= (R23-R23_cmd)/taw;
	ptr->p_cmd = 1/R33 * (R21*b_x_dot_cmd - R11*b_y_dot_cmd);
	ptr->q_cmd = 1/R33 * (R22*b_x_dot_cmd - R12*b_y_dot_cmd);
}

void vdYawBlock(parameters* ptr)
{
	ptr->r_cmd = kp_yaw*(ptr->psi_cmd - ptr->psi);
}

void vdAltitudeBlock(parameters* ptr)
{
	f32 R33 = cos(ptr->phi)/cos(ptr->theta);
	ptr->z_dot_dot_cmd= kp_z*(ptr->z_cmd- ptr->z) + kd_z*(ptr->z_dot_cmd-ptr->z_dot);
	ptr->u1 = m * (ptr->z_dot_dot_cmd - g)/R33;
}

void vdLateralBlock(parameters* ptr)
{
	ptr->x_dot_dot_cmd= kp_xy*(ptr->x_cmd- ptr->z) + kd_xy*(ptr->x_dot_cmd-ptr->x_dot);
	ptr->y_dot_dot_cmd= kp_xy*(ptr->y_cmd- ptr->z) + kd_xy*(ptr->y_dot_cmd-ptr->y_dot);
}
void fview(uint8_t type, float argument, char * line)
{
	uint8_t buffer[25];
	if(type == PRINT_FLOAT_NO_TAB || type == PRINT_FLOAT_WITH_TAB)						//0 for printing variables, else for simple print
	{
		int32_t x = argument *100;
		uint32_t y = abs(x%100);
		if(argument < 0 && x/100 >= 0 && x/100 < 1 )
		{
			(type == PRINT_FLOAT_NO_TAB)? sprintf((char*)buffer, "%s-%ld.%02lu\r\n", line, x/100,y) : sprintf((char*)buffer, "%s-%ld.%02lu\t", line, x/100,y);
		}
		else
		{
			(type == PRINT_FLOAT_NO_TAB)? sprintf((char*)buffer, "%s%ld.%02lu\r\n", line, x/100,y) : sprintf((char*)buffer, "%s%ld.%02lu\t", line, x/100,y);
		}
	}
	else if(type == PRINT_INT_NO_TAB || type == PRINT_INT_WITH_TAB)
	{
		(type == PRINT_INT_NO_TAB)? sprintf((char*)buffer, "%s%.02f\r\n", line,argument) : sprintf((char*)buffer, "%s%.02f\t", line, argument);
		sprintf((char*)buffer, line, (int32_t)argument);
	}
	else
	{
		sprintf((char*)buffer,line);
	}
	HAL_UART_Transmit(&huart1, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
}

void string_receive(char* buffer)
{
	int i = 0;
	HAL_UART_Receive(&huart1, (uint16_t*)&buffer[i], 1, HAL_MAX_DELAY);
	while(buffer[i]!='#')
	{
		i++;
		HAL_UART_Receive(&huart1, (uint8_t*)&buffer[i], 1, HAL_MAX_DELAY);
	}
	buffer[i] = '\0';
}

void vInitPARAMETERS(parameters *ptr)
{
	/***********************************/
	/*TO INITIALIZE ALL PARAMETERS TO 0*/
	/***********************************/
	ptr->x = ptr->y = ptr->z = 0;
	ptr->x_dot = ptr->y_dot = ptr->z_dot = 0;
	ptr->x_dot_dot = ptr->y_dot_dot = ptr->z_dot_dot = 0;
	ptr->phi = ptr->theta = ptr->psi = 0;
	ptr->phi_dot = ptr->theta_dot = ptr->psi_dot = 0;
	ptr->p = ptr->q = ptr->r = 0;
	ptr->p_dot = ptr->q_dot = ptr->r_dot = 0;
	ptr->x_cmd = ptr->y_cmd = ptr->z_cmd = 0;
	ptr->x_dot_cmd = ptr->y_dot_cmd = ptr->z_dot_cmd = 0;
	ptr->x_dot_dot_cmd = ptr->y_dot_dot_cmd = ptr->z_dot_dot_cmd = 0;
	ptr->psi_cmd = ptr->p_cmd = ptr->q_cmd = ptr->r_cmd = 0;
	ptr->u1 = ptr->u2 = ptr->u3 = ptr->u4 = 0;
	ptr->cmd_thrust[0] = 0;
	ptr->cmd_thrust[1] = 0;
	ptr->cmd_thrust[2] = 0;
	ptr->cmd_thrust[3] = 0;
	ptr-> phib=ptr-> thetab=ptr-> psib=0;
}
