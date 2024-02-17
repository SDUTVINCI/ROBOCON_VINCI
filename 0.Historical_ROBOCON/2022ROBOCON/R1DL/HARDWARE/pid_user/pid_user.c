#include "pid_user.h"
#include "pid.h"
#include "can.h"
#include "usart.h"


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
float motor_speed_2006_pid[3] = {8.3,0.1,1};//底盘2006参数
float motor_position_2006_pid[3] = {0.27,0.022,0.3};
float zhuashou_speed_3508_pid[3] = {14,0.20,0};//抓手3508参数
float zhuashou_position_3508_pid[3] = {0.33,0.03,0.5};
float canpan_speed_3508_pid[3] = {10,0.3,0};//餐盘3508参数
float canpan_position_3508_pid[3] = {0.3,0.1,0};
float moca_speed_3508_pid_y[3] = {19,0.19,4.3};//摩擦轮3508参数
float moca_position_3508_pid_y[3] = {0.3,0,1};
float moca_speed_3508_pid_z[3] = {19,0.22,4.3};//摩擦轮3508参数
float moca_position_3508_pid_z[3] = {0.3,0,1};

extern motor_measure_t motor_can1[8],motor_can2[8];

double angle_motor;

//底盘电机PID初始化
void pid_chassis_init(void)
{
	for(int i=0;i<4;i++)
	{
    PID_init(&pid_v[i], PID_POSITION, motor_speed_2006_pid, 9000, 6000);
		PID_init(&pid_pos[i], PID_POSITION, motor_position_2006_pid, 10000, 2000);
	}
	for(int i=4;i<8;i++)
	{		
    PID_init(&pid_v[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		PID_init(&pid_pos[i], PID_POSITION, motor_position_3508_pid, 400, 300);
	}
}

//其他电机PID初始化
void pid_other_init(void){
	  //抓手
	  PID_init(&pid_v_2[0], PID_POSITION, zhuashou_speed_3508_pid, 9000, 6000);
		PID_init(&pid_pos_2[0], PID_POSITION, zhuashou_position_3508_pid, 500, 3000);
	  //餐盘
	  PID_init(&pid_v_2[1], PID_POSITION, canpan_speed_3508_pid, 3000, 2000);
		PID_init(&pid_pos_2[1], PID_POSITION, canpan_position_3508_pid, 400, 300);
	 //摩擦轮
	  PID_init(&pid_v_2[2], PID_POSITION, moca_speed_3508_pid_y, 10000, 6000);
		PID_init(&pid_pos_2[2], PID_POSITION, moca_position_3508_pid_y, 8000, 6000);
	  PID_init(&pid_v_2[3], PID_POSITION, moca_speed_3508_pid_y, 10000, 6000);
		PID_init(&pid_pos_2[3], PID_POSITION, moca_position_3508_pid_y, 8000, 6000);
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

 double angle_motor_t_3508(double angle_rc)
{
		return	angle_motor = angle_rc/360*157293;
}



