#include "pid.h"

/**
 * @brief       PID���ĳ�ʼ��
 * @param       pid��PID���������ݴ�ŵľ��
 * @param       mode��PIDģʽ��Ϊö��@PID_MODE
 * @param       PID[3]��Kpid�������������飬��ֱ�������׵�ַ
 * @param       max_out��������޷�
 * @param       max_iout�������޷�
 * @retval      void
 * @note        ����������У�@pid_type_def����Ǳ������˲���������PIDģʽ��6������
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
 * @brief       PID���ļ��㺯��
 * @param       pid��PID���������ݴ�ŵľ��
 * @param       ref������ֵ
 * @param       set������ֵ(Ŀ��ֵ)
 * @retval      fp32���ֵ
 * @note        �ú���������м�ֵ�����������浽@pid_type_def����С�
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
 * @brief       PID��0����
 * @param       pid��PID���������ݴ�ŵľ��
 * @retval      void
 * @note        �ú������ǰ�@pid_type_def��������г�Ա�������㡣
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


