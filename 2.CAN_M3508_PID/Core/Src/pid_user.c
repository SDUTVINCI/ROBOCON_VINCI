#include "pid_user.h"



#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


pid_type_def pid_v[8],pid_pos[8];
pid_type_def pid_v_2[8],pid_pos_2[8];

float motor_speed_3508_pid[3] = {10, 0.1, 0};//底盘3508参数
float motor_position_3508_pid[3] = {0.2, 0, 1};

extern motor_measure_t motor_can1[8];


//底盘电机PID初始化
void pid_chassis_init(void)
{
	for(int i=0;i<4;i++)
	{
    PID_init(&pid_v[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		PID_init(&pid_pos[i], PID_POSITION, motor_position_3508_pid, 400, 300);
	}
	
	for(int i=4;i<8;i++)
	{		
    PID_init(&pid_v[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		PID_init(&pid_pos[i], PID_POSITION, motor_position_3508_pid, 400, 300);
	}
}



float PID_velocity_realize_1(float set_speed,int i)
{
		PID_calc(&pid_v[i-1],motor_can1[i-1].speed_rpm , set_speed);
		return pid_v[i-1].out;
}

float PID_position_realize_1(float set_pos,int i)
{

		PID_calc(&pid_pos[i-1],motor_can1[i-1].total_angle , set_pos);
		return pid_pos[i-1].out;

}

float pid_call_1(float position,int i)
{
		return PID_velocity_realize_1(PID_position_realize_1(position,i),i);
}


