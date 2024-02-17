#ifndef PID_USER_H
#define PID_USER_H
#include "SYS.h"

void pid_chassis_init(void);
void pid_other_init(void);

float PID_velocity_realize_1(float set_speed,int i);
float PID_position_realize_1(float set_pos,int i);
float pid_call_1(float position,int i);
float PID_velocity_realize_2(float set_speed,int i);
float PID_position_realize_2(float set_pos,int i);
float pid_call_2(float position,int i);
float PID_YAW_POS_realize(float set_yaw);
float PID_X_POS_realize(float set_x);
float PID_Y_POS_realize(float set_y);
double angle_motor_t_2006(double angle_rc);
double angle_motor_t_3508(double angle_rc);
double angle_motor_t_xiebo(double angle_rc);

#endif























