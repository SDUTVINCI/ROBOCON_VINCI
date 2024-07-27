#ifndef __PID_USER_H
#define __PID_USER_H
#include "pid.h"
#include "include.h"

void PID_devices_Init(void);
fp32 PID_velocity_realize_chassis_3508(fp32 set_speed,int i);
//fp32 PID_velocity_realize_chassis_2006(fp32 set_speed,int i);
//fp32 PID_position_realize_chassis_2006(fp32 set_pos,int i);
fp32 PID_call_2006(fp32 position,int i);
fp32 PID_call_3508(fp32 position,int i);//can1 2006 chalss




fp32 PID_velocity_realize_2_3508(fp32 set_speed,int i);


fp32 PID_position_realize_2_3508(fp32 set_pos,int i);

fp32 PID_call_2_3508(fp32 position,int i);

fp32 PID_velocity_realize_2_2006(fp32 set_speed,int i);

fp32 PID_position_realize_2_2006(fp32 set_pos,int i);

fp32 PID_call_2_2006(fp32 position,int i);
#endif
