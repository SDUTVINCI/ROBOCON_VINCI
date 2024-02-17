#ifndef __CHASSIS_API_H_
#define __CHASSIS_API_H_
#include "include.h"

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
} Chassis_Speed_t;

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
		fp32 yaw_last_angle;
		fp32 yaw_rel;   //yaw_relative
		fp32 yaw_round_cnt;
		fp32 yaw_total;
		
		fp32 yaw_target;
	}Euler;
	
	__packed struct
	{
		fp32 Pos_X;
		fp32 Pos_Y;
		
		fp32 Pos_X_offset;
		fp32 Pos_Y_offset;
		
		fp32 Pos_X_Target;
		fp32 Pos_Y_Target;
	}Position;

}Chassis_measure_t;

void Remote_Control_Chassis_Set_Mode(void);

void Remote_Control_Chassis_Mode(Chassis_Speed_t *chassis_speed);

void Chassis_Pose_Analysis(void);

void Chassis_Sports_Calc(Chassis_Speed_t speed);

void Chassis_Loop_Out(void);

#endif
