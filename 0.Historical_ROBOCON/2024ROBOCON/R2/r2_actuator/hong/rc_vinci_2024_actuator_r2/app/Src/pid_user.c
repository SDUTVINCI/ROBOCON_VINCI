#include "pid_user.h"
#include "can_receive.h"

extern motor_measure_t motor_can1[8];
extern moto_info_t motor_info[MOTOR_MAX_NUM];

pid_type_def pid_v_chassis_3508[8],
    pid_v_chassis_2006[8],
    pid_p_chassis_2006[8],
    pid_track_ball_x,
    pid_track_ball_distance;

float motor_speed_3508_pid[3] = {20, 0.1, 0}; // 底盘3508参数
float motor_position_3508_pid[3] = {0.2, 0, 1};

float chassis_track_ball_x_pid[3] = {5, 0, 0.3};
float chassis_track_ball_distance_pid[3] = {5, 0, 0};
float motor_speed_2006_pid[3] = {8.3, 0.1, 1}; // 底盘2006参数
float motor_position_2006_pid[3] = {0.27, 0.022, 0.7};
float motor_position_2006_pid2[3] = {0.5, 0, 0};
#define LimitMax(input, max)                                  \
	{                                             \
		if (input > max)              \
		{                             \
			input = max;  \
		}                             \
		else if (input < -max)        \
		{                             \
			input = -max; \
		}                             \
	}

// 底盘电机PID初始化
void PID_devices_Init(void)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		PID_init(&pid_v_chassis_3508[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000); // 3508  speed
	}
	PID_init(&pid_track_ball_x, PID_POSITION, chassis_track_ball_x_pid, 2000, 500);
	PID_init(&pid_track_ball_distance, PID_POSITION, chassis_track_ball_distance_pid, 2000, 500);
}
extern int16_t dx;
// extern int16_t distance_carmere;

int16_t PID_chassis_track_ball_x(int16_t x)
{
	PID_calc(&pid_track_ball_x, (float)x, 0.0);
	return pid_track_ball_x.out;
}
int16_t PID_chassis_track_ball_distance(int16_t distance)
{
	PID_calc(&pid_track_ball_distance, (float)distance, 0.0);
	return pid_track_ball_distance.out;
}

fp32 PID_velocity_realize_chassis_3508(fp32 set_speed, int i) // can1 3508 speed
{
	PID_calc(&pid_v_chassis_3508[i - 1], motor_can1[i - 1].speed_rpm, set_speed);
	return pid_v_chassis_3508[i - 1].out;
}

fp32 PID_velocity_realize_chassis_2006(fp32 set_speed, int i) // can1 2006 chalss 456
{
	PID_calc(&pid_v_chassis_2006[i - 1], motor_can1[i - 1].speed_rpm, set_speed);
	return pid_v_chassis_2006[i - 1].out;
}

fp32 PID_position_realize_chassis_2006(fp32 set_pos, int i) // can1 2006 chalss
{

	PID_calc(&pid_p_chassis_2006[i - 1], motor_can1[i - 1].total_angle, set_pos);
	return pid_p_chassis_2006[i - 1].out;
}

fp32 PID_call_2006(fp32 position, int i) // can1 2006 chalss
{
	return PID_velocity_realize_chassis_2006(PID_position_realize_chassis_2006(position, i), i);
}
