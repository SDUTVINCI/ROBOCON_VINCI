#ifndef __PID_USER_H
#define __PID_USER_H
#include "pid.h"
#include "include.h"

void PID_devices_Init(void);

fp32 PID_velocity_realize_1(fp32 set_speed,int i);
fp32 PID_position_realize_1(fp32 set_pos,int i);
fp32 PID_call_1(fp32 position,int i);

fp32 PID_velocity_realize_2(fp32 set_speed,int i);
fp32 PID_position_realize_2(fp32 set_pos,int i);
fp32 PID_call_2(fp32 position,int i);

fp32 PID_Yaw_realize(fp32 set_yaw);
fp32 PID_Pos_X_realize(fp32 set_pos_x);
fp32 PID_Pos_Y_realize(fp32 set_pos_y);


#endif























