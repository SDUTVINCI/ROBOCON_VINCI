#include "pid_user.h"

extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];

pid_type_def pid_v_1[8],pid_pos_1[8];
pid_type_def pid_v_2[8],pid_pos_2[8];

float motor_speed_3508_pid[3] = {10, 0.1, 0};//底盘3508参数
float motor_position_3508_pid[3] = {0.2, 0, 1};
float motor_speed_2006_pid[3] = {8.3,0.1,1};//底盘2006参数
float motor_position_2006_pid[3] = {0.27,0.022,0.3};


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


//底盘电机PID初始化
void PID_devices_Init(void)
{
	for(int i=0;i<4;i++)
	{
    PID_init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		PID_init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid, 400, 300);
		
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 9000, 6000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 10000, 2000);
	}
	
	for(int i=4;i<8;i++)
	{		
    PID_init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		PID_init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid, 400, 300);
		
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid, 400, 300);
	}
}



float PID_velocity_realize_1(float set_speed,int i)
{
		PID_calc(&pid_v_1[i-1],motor_can1[i-1].speed_rpm , set_speed);
		return pid_v_1[i-1].out;
}

float PID_position_realize_1(float set_pos,int i)
{

		PID_calc(&pid_pos_1[i-1],motor_can1[i-1].total_angle , set_pos);
		return pid_pos_1[i-1].out;

}

float pid_call_1(float position,int i)
{
		return PID_velocity_realize_1(PID_position_realize_1(position,i),i);
}






float PID_velocity_realize_2(float set_speed,int i)
{
		PID_calc(&pid_v_2[i-1],motor_can2[i-1].speed_rpm , set_speed);
		return pid_v_2[i-1].out;
}

float PID_position_realize_2(float set_pos,int i)
{

		PID_calc(&pid_pos_2[i-1],motor_can2[i-1].total_angle , set_pos);
		return pid_pos_2[i-1].out;

}

float pid_call_2(float position,int i)
{
		return PID_velocity_realize_2(PID_position_realize_2(position,i),i);
}
