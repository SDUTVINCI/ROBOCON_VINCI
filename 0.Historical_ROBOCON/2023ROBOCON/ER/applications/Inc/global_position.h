#ifndef __GLOBAL_POSITION_H_
#define __GLOBAL_POSITION_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "imu_ops9.h"


typedef struct
{
	__packed struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Accel;

	__packed struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Gyro;
	
	__packed struct
	{
		fp32 X;
		fp32 Y;
		fp32 Z;
	}Magnet;
	
	__packed struct
	{
		fp32 yaw;
		fp32 pitch;
		fp32 roll;
	}Euler;
	
	__packed struct
	{
		fp32 quat[4];
	}Quaternion;
	
	__packed struct
	{
		fp32 Pos_X;
		fp32 Pos_Y;
	}Position;
	
	__packed struct
	{
		fp32 temp;
	}Other_Data;
	
}IMU_t;

void Get_Chassis_Yaw_Target(void);

void Get_Chassis_Yaw_Offset(void);

void Get_Chassis_Total_Yaw(void);

void Yaw_PID_Calc(fp32 atan2_angle);

void OPS9_Reset_Origin(UART_HandleTypeDef *huart);

void OPS9_Set_Coordinate(fp32 X_Target,fp32 Y_Target,fp32 Yaw_Target);

fp32 OSP9_Angle_Target_Calc(void);
	
void OPS9_GP_Stage1(void);

void New_Helm_Angle_Calc(void);

void Loop_PID_CMD(void);

fp32 deg2rad(fp32 deg);

fp32 rad2deg(fp32 rad);



#ifdef __cplusplus
}
#endif

#endif

