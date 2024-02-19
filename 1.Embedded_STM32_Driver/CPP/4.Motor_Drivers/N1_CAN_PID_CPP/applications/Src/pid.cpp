#include "pid.h"

/**
 * @brief       PID核心初始化
 * @param       pid：PID参数及数据存放的句柄
 * @param       mode：PID模式，为枚举@PID_MODE
 * @param       PID[3]：Kpid三个参数的数组，可直接填其首地址
 * @param       max_out：总输出限幅
 * @param       max_iout：积分限幅
 * @retval      void
 * @note        在这个函数中，@pid_type_def句柄是被赋予了参数，包括PID模式等6个参数
 */
void PID_Controller::CORE::PID_Init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == 0 || PID == 0)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief       PID核心计算函数
 * @param       pid：PID参数及数据存放的句柄
 * @param       ref：反馈值
 * @param       set：期望值(目标值)
 * @retval      fp32输出值
 * @note        该函数计算的中间值与结果都将保存到@pid_type_def句柄中。
 */
fp32 PID_Controller::CORE::PID_Calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}


/**
 * @brief       PID清0函数
 * @param       pid：PID参数及数据存放的句柄
 * @retval      void
 * @note        该函数是是把@pid_type_def句柄的所有成员变量清零。
 */
void PID_Controller::CORE::PID_Clear(pid_type_def *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}


