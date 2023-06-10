#ifndef __CHASSIS_API_H_
#define __CHASSIS_API_H_
#include "include.h"

#define PI (3.141592f)

//底盘模式
typedef enum
{
    CHASSIS_GYROSCOPE = 0,	   //小陀螺模式
    CHASSIS_NORMAL   = 1,      //底盘遥控行走
} eChassisAction;


//底盘速度
typedef struct
{
    fp32 vx;
    fp32 vy;
    fp32 vw;
}Chassis_Speed_t;

typedef struct
{
	__packed struct
	{
		fp32 vx;
		fp32 vy;
		fp32 vw;
	}Speed;
	
	__packed struct
	{
		fp32 yaw;
		fp32 pitch;
		fp32 roll;
		
		fp32 yaw_offset;
		fp32 yaw_rel;   //yaw_relative
		
		fp32 yaw_target;
	}Euler;
	
}Chassis_measure_t;

void Remote_Control_Chassis_Set_Mode(void);

void Remote_Control_Chassis_Mode(Chassis_Speed_t *chassis_speed);

void Chassis_Sports_Calc(Chassis_Speed_t speed);

void Chassis_Loop_Out(void);

fp32 deg2rad(fp32 deg);

fp32 rad2deg(fp32 rad);

#endif
