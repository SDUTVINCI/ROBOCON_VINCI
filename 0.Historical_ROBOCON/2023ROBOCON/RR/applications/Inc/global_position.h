#ifndef __GLOBAL_POSITION_H_
#define __GLOBAL_POSITION_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

#define PI (3.1415926f)


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

void Yaw_PID_Calc(void);

fp32 deg2rad(fp32 deg);

fp32 rad2deg(fp32 rad);



#ifdef __cplusplus
}
#endif

#endif

