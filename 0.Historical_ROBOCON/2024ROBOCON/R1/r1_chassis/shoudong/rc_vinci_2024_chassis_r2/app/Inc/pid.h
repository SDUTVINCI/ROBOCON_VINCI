#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"
#include "include.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_calc_6020_postion(pid_type_def *pid, float ref, float set);
extern float PID_calc(pid_type_def *pid, float ref, float set);
 
extern void PID_clear(pid_type_def *pid);

#endif



