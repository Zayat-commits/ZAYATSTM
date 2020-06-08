/*
 * Blocks.c
 *
 *  Created on: May 20, 2020
 *      Author: abdul
 */

#include "main.h"
extern f32 Xh,Yh;
extern accel mag;
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
	s8 buffer[25];
	s8 coordinates[][3] = {{"X"},{"Y"},{"Z"}};
	s8 velocities[][15] = {{"X VELOCITY"},{"Y VELOCITY"},{"Z VELOCITY"}};
	u8 next;
	menu:
	next = 0;
	vdFrames("STARS");
	fview(PRINT_NORMAL, 0, "----------------------------------------DRONE TESTING PLATFORM----------------------------------------------\n \n");
	vdFrames("STARS");
	fview(PRINT_NORMAL, 0, "TO INSERT PARAMETER VALUES INTO THE DRONE: FOLLOW THE GUIDE BELOW\n \n");
	do
	{
		fview(PRINT_NORMAL, 0, "FOR TARGET X Y Z: INSERT 1 \nFOR TARGET X Y Z VELOCITIES: INSERT 2 \nFOR TARGET PSI: INSERT 3 \nTO RETURN TO PREVIOUS MENU INSERT 0\r\n");
		string_receive((u8*)buffer);
	}while(atoi(buffer) < 0 || atoi(buffer) > 3);
	volatile int temp = atoi(buffer);

	switch(temp)
	{
	case 1:
	{
		switchzayat:
		switch (u8TestUserInterface(coordinates[next], ptr))
		{
			case 0:
			{
				break;
			}
			case 1:
			{
				goto menu;
			}
			case 2:
			{
				next++;
				if(next > 2)break;
				goto switchzayat;
			}
		}
		next = 0;
		break;
	}
	case 2:
	{
		switchzayat1:
		switch (u8TestUserInterface(velocities[next], ptr))
		{
			case 0:
			{
				break;
			}
			case 1:
			{
				goto menu;
			}
			case 2:
			{
				next++;
				if(next > 2)break;
				goto switchzayat1;
			}
		}
		break;
	}
	case 3:
	{
		switch (u8TestUserInterface("PSI", ptr))
		{
			case 0:
			{
				break;
			}
			case 1:
			{
				goto menu;
			}
			case 2:
				break;
		}
		break;
	}
	case 0:
	{
		ptr->ret_flag = 1;
		break;
	}
}
}
void vdMPUBlock(parameters* ptr)
{
	/*Read Gyro and Accel values, then comp filter*/
	Read_Accel_Values(ptr);
	Read_Gyro_Values(ptr,INTEGRAL_DT);
	imu_Comp_Filter(ptr,INTEGRAL_DT);
	Read_Compass_Values(ptr);


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
	u8 i;
	f32 speed_pwm[4]={0};
	for(i=0 ; i<4;i++)
	{
		if (ptr->cmd_thrust[i] < F_min) ptr->cmd_thrust[i]=F_min;
		if (ptr->cmd_thrust[i] > F_max) ptr->cmd_thrust[i]=F_max;
		speed_pwm[i] = (1/54)*(ptr->cmd_thrust[i])*25  + 40.0;
//		fview(PRINT_NORMAL, 0, "speed of motor: ");
//		fview(PRINT_FLOAT_NO_TAB, speed_pwm[i], " = ");
		PWM(speed_pwm[i],i+1);
	}
}

void vdRollPitchBlock(parameters* ptr)
{
	f32 b_x_dot_cmd, b_y_dot_cmd, taw=1/kp_bank;
	f32 R11 = 1;
	f32 R12 = sin(ptr->phi *DEG_TO_RAD) * sin(ptr->theta *DEG_TO_RAD) / cos(ptr->theta *DEG_TO_RAD);
	f32 R13= cos(ptr->phi *DEG_TO_RAD) * sin(ptr->theta *DEG_TO_RAD) / cos(ptr->theta *DEG_TO_RAD);
	f32 R21 = 0;
	f32 R22 = cos(ptr->phi*DEG_TO_RAD);
	f32 R23= -sin(ptr->phi*DEG_TO_RAD);
	f32 R31 = 0;
	f32 R32 = sin(ptr->phi*DEG_TO_RAD) / cos(ptr->theta*DEG_TO_RAD);
	f32 R33 = cos(ptr->phi*DEG_TO_RAD) / cos(ptr->theta*DEG_TO_RAD);
	f32 R13_cmd= ptr->x_dot_dot_cmd*mass/ptr->u1;
	f32 R23_cmd= ptr->y_dot_dot_cmd*mass/ptr->u1;
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
	f32 R33 = cos(ptr->phi*DEG_TO_RAD)/cos(ptr->theta*DEG_TO_RAD);
	ptr->z_dot_dot_cmd= kp_z*(ptr->z_cmd- ptr->z) + kd_z*(ptr->z_dot_cmd-ptr->z_dot);
	ptr->u1 = mass * (ptr->z_dot_dot_cmd - g)/R33;
}

void vdLateralBlock(parameters* ptr)
{
	ptr->x_dot_dot_cmd= kp_xy*(ptr->x_cmd- ptr->z) + kd_xy*(ptr->x_dot_cmd-ptr->x_dot);
	ptr->y_dot_dot_cmd= kp_xy*(ptr->y_cmd- ptr->z) + kd_xy*(ptr->y_dot_cmd-ptr->y_dot);
}
void fview(uint8_t type, float argument, char * line)
{
	uint8_t buffer[150];
//	  __HAL_UNLOCK(&huart1);
	if(type == PRINT_FLOAT_NO_TAB || type == PRINT_FLOAT_WITH_TAB)						//0 for printing variables, else for simple print
	{
		int32_t x = argument *100;
		uint32_t y = abs(x%100);
		if(argument < 0 && x/100 >= 0 && x/100 < 1 )
		{
			(type == PRINT_FLOAT_NO_TAB)? sprintf((char*)buffer, "%s-%ld.%02lu \n", line, x/100,y) : sprintf((char*)buffer, "%s-%ld.%02lu \t", line, x/100,y);
		}
		else
		{
			(type == PRINT_FLOAT_NO_TAB)? sprintf((char*)buffer, "%s%ld.%02lu \n", line, x/100,y) : sprintf((char*)buffer, "%s%ld.%02lu \t", line, x/100,y);
		}
	}
	else if(type == PRINT_INT_NO_TAB || type == PRINT_INT_WITH_TAB)
	{
		(type == PRINT_INT_NO_TAB)? sprintf((char*)buffer, "%s%d \n", line,(int)argument) : sprintf((char*)buffer, "%s%d \t", line, (int)argument);
	}
	else
	{
		sprintf((char*)buffer,line);
	}
	HAL_UART_Transmit(&huart1, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
}

void string_receive(u8* buffer)
{
	int i = 0;
	HAL_UART_Receive(&huart1, &buffer[i], 1, HAL_MAX_DELAY);
	while(buffer[i]!='\n')
	{
		i++;
		HAL_UART_Receive(&huart1, &buffer[i], 1, HAL_MAX_DELAY);
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
	ptr->u1 =1;
	ptr->u2 = ptr->u3 = ptr->u4 = 0;
	ptr->cmd_thrust[0] = 0;
	ptr->cmd_thrust[1] = 0;
	ptr->cmd_thrust[2] = 0;
	ptr->cmd_thrust[3] = 0;
	ptr->motor1 = ptr->motor2 = ptr->motor3 = ptr->motor4 = 0;
	ptr->status.pwm = PWM_OFF;
	ptr-> phib=ptr-> thetab=ptr-> psib=0;
}

void vdUserInterface(void)
{
	vdFrames("SPRINKLES");
	fview(PRINT_NORMAL, 0, "+*----------------------------------RAVEN5 DRONE USER INTERFACE-------------------------------------------+*\n \n");
	vdFrames("SPRINKLES");
	fview(PRINT_NORMAL, 0, "SELECT MODE OF OPERATION BY INSERTING MODE NUMBER:\nMODE 0: CALIBRATION OF MOTORS \nMODE 1: NORMAL START OF MOTORS \nMODE 2: FREE CONTROL OF MOTORS\n");
	fview(PRINT_NORMAL, 0, "MODE 3: HOVER POWER OF MOTORS \nMODE 4: FULL TEST OF SYSTEM WITH MOTORS \nMODE 5: FULL TEST OF SYSTEM WITHOUT MOTORS \n");
	fview(PRINT_NORMAL, 0, "MODE 6: GET STATUS \nMODE 7: PROGRAM START \nMODE 8: SYSTEM RESET \n");
}
u8 u8TestUserInterface(const char* line,parameters* ptr )		/*returns value of what the user wants to do: Insert, Return or Exit*/
{
	s8 buffer1[150];
	s8 buffer[25];
	sprintf(buffer1,"INSERT TARGET %s (-128 ~ 127), 'return' TO DISPLAY CURRENT MENU OR 'exit' TO GO TO MAIN MENU\n", line);
	insert:
	fview(PRINT_NORMAL, 0, buffer1);
	string_receive((u8*)buffer);
	if (strcmp((char*)buffer,"exit") == 0)
	{
		do
			{
				fview(PRINT_NORMAL, 0, "EXIT?\n 1: YES \t 2: NO\n");
				string_receive((u8*)buffer);
			}while(strcmp(buffer,"1") !=0 && strcmp(buffer,"2") !=0);
		if(strcmp(buffer, "1") ==0)
			{
				ptr->ret_flag = 1;
				return 0;
			}
		else
			{
				goto insert;
			}
	}
	else if(strcmp((char*)buffer,"return") == 0)
	{
		do
			{
				fview(PRINT_NORMAL, 0, "RETURN?\n 1: YES \t 2: NO\n");
				string_receive((u8*)buffer);
			}while(strcmp(buffer,"1") !=0 && strcmp(buffer,"2") !=0);
		if(strcmp(buffer, "1") ==0)return 1;
		if(strcmp(buffer, "2") ==0)goto insert;
	}
	else
	{
		do
			{
				fview(PRINT_INT_NO_TAB, atoi(buffer), "\t");
				fview(PRINT_NORMAL, 0, "INSERT VALUE?\n 1: YES \t 2: NO\n");
				string_receive((u8*)buffer);
			}while(strcmp(buffer,"1") !=0 && strcmp(buffer,"2") !=0);
		if(strcmp(buffer, "2") ==0)goto insert;
		if(strcmp(buffer, "1") ==0)ptr->x_cmd = atoi(buffer);
		return 2;
	}
	return -1;

}
void vdFrames(const char* type)
{
	if(strcmp(type, "STARS")==0)
	{
		fview(PRINT_NORMAL, 0, "*********************************************************************************************************** \n \n");
	}
	else if(strcmp(type, "SPRINKLES")==0)
	{
		fview(PRINT_NORMAL, 0, "+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*++*+*+*+*+*+*+*+**+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*\n \n");
	}
}
