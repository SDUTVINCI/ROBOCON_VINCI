#ifndef __PID_USER_H
#define __PID_USER_H
#include "pid.h"
#include "include.h"

void PID_devices_Init(void);
fp32 PID_velocity_realize_6020(fp32 set_speed, int i);
fp32 PID_velocity_realize_chassis_3508(fp32 set_speed, int i);
fp32 PID_position_6020(fp32 set_pos, int i);
fp32 PID_call_6020(fp32 position, int i);
fp32 PID_position_mapan_x(fp32 set_position);
fp32 PID_position_mapan_y(fp32 set_position);
fp32 PID_position_L1S_x(int set_position);
fp32 PID_position_L1S_y(int set_position);
fp32 PID_position_L1S(int set_position, int now_position);
fp32 PID_position_chassis_omega1(fp32 set_omega); // position
fp32 PID_position_chassis_omega2(fp32 set_omega); // position

fp32 PID_position_Mapan_y(int set_position);
fp32 PID_position_L1S_jiaozhun(void); // position

fp32 PID_position_Mapan_x(int set_position);
#endif
