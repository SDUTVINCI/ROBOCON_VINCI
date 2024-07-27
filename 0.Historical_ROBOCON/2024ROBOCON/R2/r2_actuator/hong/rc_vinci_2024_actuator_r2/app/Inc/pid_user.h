#ifndef __PID_USER_H
#define __PID_USER_H
#include "pid.h"
#include "include.h"

void PID_devices_Init(void);
fp32 PID_velocity_realize_chassis_3508(fp32 set_speed, int i);
fp32 PID_velocity_realize_chassis_2006(fp32 set_speed, int i);
fp32 PID_position_realize_chassis_2006(fp32 set_pos, int i);
int16_t PID_chassis_track_ball_x(int16_t x);
int16_t PID_chassis_track_ball_distance(int16_t distance);

fp32 PID_call_2006(fp32 position, int i);
#endif
