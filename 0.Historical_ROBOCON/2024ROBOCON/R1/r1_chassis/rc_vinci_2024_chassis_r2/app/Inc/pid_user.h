#ifndef __PID_USER_H
#define __PID_USER_H
#include "pid.h"
#include "include.h"

void PID_devices_Init(void);
 fp32 PID_velocity_realize_6020(fp32 set_speed,int i);
fp32 PID_velocity_realize_chassis_3508(fp32 set_speed,int i);
fp32 PID_position_6020(fp32 set_pos,int i);
 fp32 PID_call_6020(fp32 position,int i);
 fp32 PID_call_omega(fp32 position);
 fp32 PID_position_chassis_x(fp32 set_position);
 fp32 PID_position_chassis_y(fp32 set_position );
 fp32 PID_position_chassis_omega(fp32 set_omega );//position
 fp32 PID_position_omega_speed(fp32 set_speed );//position
fp32 PID_position_L1S_X1(int set_position); // position
fp32 PID_position_L1S_X2(int set_position) ;// position
fp32 PID_position_L1S_Y1(int set_position) ;// position
fp32 PID_position_L1S_Y2(int set_position) ;// position
#endif























