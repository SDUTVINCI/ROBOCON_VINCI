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
		
pid_type_def pid_yaw,pid_x,pid_y;

float motor_speed_3508_pid[3] = {14, 0.3, 1};
float motor_position_3508_pid[3] = {0.2, 0, 1};
float motor_speed_2006_pid[3] = {9, 0.1, 0};
float motor_position_2006_pid[3] = {0.2, 0, 0};
float yaw_pid[3] = {70, 0, 700};
float x_pid[3] = {20, 0, 500};
float y_pid[3] = {20, 0, 500};


extern float yaw;
extern float len_back,len_right;

extern motor_measure_t motor_can1[8],motor_can2[8];

double angle_motor;

void pid_chassis_init(void)
{
	for(int i=0;i<8;i++)
	{
    PID_init(&pid_v[i], PID_POSITION, motor_speed_3508_pid, 9000, 6000);
		PID_init(&pid_pos[i], PID_POSITION, motor_position_3508_pid, 400, 300);
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid, 9000, 8000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid, 8000, 6000);

	}
		PID_init(&pid_v_2[0], PID_POSITION, motor_speed_2006_pid, 16000, 16000);
		PID_init(&pid_pos_2[0], PID_POSITION, motor_position_2006_pid, 16000, 16000);
		PID_init(&pid_yaw, PID_POSITION, yaw_pid, 1000, 800);
		PID_init(&pid_x, PID_POSITION, x_pid, 3000, 600);
		PID_init(&pid_y, PID_POSITION, y_pid, 2000, 600);
}

float PID_velocity_realize_1(float set_speed,int i)           //速度环
{
		PID_calc(&pid_v[i-1],motor_can1[i-1].speed_rpm , set_speed);
		return pid_v[i-1].out;
}

float PID_position_realize_1(float set_pos,int i)           //位置环
{

		PID_calc(&pid_pos[i-1],motor_can1[i-1].total_angle , set_pos);
		return pid_pos[i-1].out;

}

float pid_call_1(float position,int i)           //双环
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


float PID_YAW_POS_realize(float set_yaw)
{
		PID_calc(&pid_yaw, yaw , set_yaw);
		return pid_yaw.out;
}

int b = 0;

extern float pos_x,pos_y;

float PID_X_POS_realize(float set_x)
{
		PID_calc(&pid_x, pos_x , set_x);
	  if (pid_x.Dout != 0)
				b = 1;
		return pid_x.out;
}

int c = 0;
float PID_Y_POS_realize(float set_y)
{
		PID_calc(&pid_y, pos_y , set_y);
	if (pid_y.Dout != 0)
				c = 1;
		return pid_y.out;
}


double angle_motor_t_2006(double angle_rc)
{
//		return	angle_motor = angle_rc/360*1474380;
	return	angle_motor = angle_rc/360*296876;
}

 double angle_motor_t_3508(double angle_rc)
{
		return	angle_motor = angle_rc/360*157293;
}


double angle_motor_t_xiebo(double angle_rc)
{
		return	angle_motor = angle_rc/360*7864650;
}



