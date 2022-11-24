#ifndef PID_USER_H
#define PID_USER_H
#include "pid.h"
#include "can.h"

void pid_chassis_init(void);

float PID_velocity_realize_1(float set_speed,int i);
float PID_position_realize_1(float set_pos,int i);
float pid_call_1(float position,int i);

#endif























