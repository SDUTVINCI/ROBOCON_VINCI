#ifndef DUOLUN_H
#define DUOLUN_H
#include "SYS.h"

float calc_angle_duolun(float set_ch2,float set_ch3);
float calc_motor_quanshu(float angle,float last_angle);
float calc_min_angle(float set_angle,float last_set_angle);

#endif


