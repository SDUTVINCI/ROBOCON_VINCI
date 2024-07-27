#include "pid_user.h"
#include "can_receive.h"

/*kalman begen*/
#include "kalman.h"
extern KALMAN_FILTER Kalman_L1sX1, Kalman_L1sX2, Kalman_L1sY1;
/*kalman end*/

extern fp32 x_mapan;
extern fp32 y_mapan;
extern motor_measure_t motor_can1[8];
extern moto_info_t motor_info[MOTOR_MAX_NUM];
extern fp32 yaw;
pid_type_def
    pid_v_chassis_3508[3],
    pid_v_chassis_6020[3],
    pid_p_chassis_6020[3],
    pid_position_chassis_omega2,
    pid_position_chassis_omega1,
    pid_position_chassis_y,
    pid_position_chassis_x,
    pid_position_mapan_x,
    pid_position_mapan_y,
    pid_position_chassis_jiaozhun; //  0:  x方向   1：y方向

float motor_speed_3508_pid[3] = {15, 0, 0}; // 底盘3508参数
float motor_position_3508_pid[3] = {0.2, 0, 1};
// float motor_speed_2006_pid[3] = {8.3, 0.1, 1}; // 底盘2006参数
// float motor_position_2006_pid[3] = {0.27, 0.022, 0.7};
float motor_speed_6020_pid[3] = {40, 3, 0};
float motor_position_6020_pid[3] = {1, 0, 4};
float motor_position_chassis_L1S[3] = {10, 0, 0};
float motor_position_chassis_mapan[3] = {10, 0, 0.1};     // 1 0 1.5
float motor_position_chassis_omega1[3] = {0.2, 0, 0.2}; //{0.5,0,0.02};  0.14   0   0.2
float motor_position_chassis_omega2[3] = {0.05, 0, 0};   //{0.5,0,0.02};
float motor_position_chassis_jiaozhun[3] = {0.05, 0, 1}; //{0.5,0,0.02};
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

	for (uint8_t i = 0; i < 3; i++)
	{
		PID_init(&pid_v_chassis_3508[i], PID_POSITION, motor_speed_3508_pid, 10000, 0);      // 3508  speed
		PID_init(&pid_v_chassis_6020[i], PID_POSITION, motor_speed_6020_pid, 30000, 0);    // 6020 speed
		PID_init(&pid_p_chassis_6020[i], PID_POSITION, motor_position_6020_pid, 20000, 0); // 6020 position
	}
	PID_init(&pid_position_chassis_x, PID_POSITION, motor_position_chassis_L1S, 2000, 300); // 激光测距定位
	PID_init(&pid_position_chassis_y, PID_POSITION, motor_position_chassis_L1S, 2000, 300);
	PID_init(&pid_position_chassis_omega1, PID_POSITION, motor_position_chassis_omega1,7, 3);
	PID_init(&pid_position_chassis_omega2, PID_POSITION, motor_position_chassis_omega2, 10, 3);
	PID_init(&pid_position_chassis_jiaozhun, PID_POSITION, motor_position_chassis_jiaozhun, 6, 3);
	PID_init(&pid_position_mapan_x, PID_POSITION, motor_position_chassis_mapan, 2000, 300); // 码盘定位
	PID_init(&pid_position_mapan_y, PID_POSITION, motor_position_chassis_mapan, 2000, 300);
	//		PID_init(&pid_position_chassis_x, PID_POSITION, motor_position_chassis, 330, 330);
	//		PID_init(&pid_position_chassis_y, PID_POSITION, motor_position_chassis, 330, 330);
	//		PID_init(&pid_position_chassis_omega, PID_POSITION, motor_position_chassis_omega, 3, 3);
}
extern unsigned int gdistance_X;
extern unsigned int gdistance_Y;
extern unsigned int gdistance_X2;
fp32 PID_position_mapan_x(fp32 set_position) // position
{
	PID_calc(&pid_position_chassis_x, x_mapan, set_position);
	return pid_position_chassis_x.out;
}
fp32 PID_position_mapan_y(fp32 set_position) // position
{
	PID_calc(&pid_position_chassis_y, y_mapan, set_position);
	return pid_position_chassis_y.out;
}
fp32 PID_position_L1S_x(int set_position) // position
{
	int tttt = set_position - gdistance_X * cos((90 + yaw) * 3.14 / 360);
	if (abs(tttt) < 10)
	{
		return 0;
	}
	else
	{
		PID_calc(&pid_position_chassis_x, gdistance_X * cos((90 + yaw) * 3.14 / 360), set_position);
		return pid_position_chassis_x.out;
	}
}
double angle;
int att;
fp32 PID_position_L1S_jiaozhun(void) // position
{
	// int tttt = set_position - gdistance_Y * cos((90 + yaw) * 3.14 / 360);
	// if (abs(tttt) < 10)
	// {
	// 	return 0;
	// }
	// else
	// {
	// att = CalcKalmanFilter(&Kalman_L1sX1, gdistance_X) + 75 - CalcKalmanFilter(&Kalman_L1sX2, gdistance_X2);
	att = gdistance_X + 75 - gdistance_X2;
	PID_calc(&pid_position_chassis_jiaozhun, att, 0);
	return pid_position_chassis_jiaozhun.out;
	// }
}
fp32 PID_position_L1S_y(int set_position) // position
{
	// int tttt = set_position - CalcKalmanFilter(&Kalman_L1sY1, gdistance_Y) * cos((90 + yaw) * 3.14 / 360);
	int tttt = set_position - gdistance_Y * cos((90 + yaw) * 3.14 / 360);
	if (abs(tttt) < 10)
	{
		return 0;
	}
	else
	{

		PID_calc(&pid_position_chassis_y, gdistance_Y * cos((90 + yaw) * 3.14 / 360), set_position);
		return pid_position_chassis_y.out;
	}
}

fp32 PID_position_Mapan_y(int set_position) // position
{

	PID_calc(&pid_position_mapan_y, y_mapan, set_position);
	return pid_position_mapan_y.out;
}

fp32 PID_position_Mapan_x(int set_position) // position
{

	PID_calc(&pid_position_mapan_x, x_mapan, set_position);
	return pid_position_mapan_x.out;
}

fp32 PID_position_chassis_omega1(fp32 set_omega) // 在三区内使用的
{
	if (fabs(set_omega - yaw) < 0.3)
	{
		return 0;
	}
	else
	{
		PID_calc(&pid_position_chassis_omega1, yaw, set_omega);
		return pid_position_chassis_omega1.out;
	}
}

fp32 PID_position_chassis_omega2(fp32 set_omega) // 到达三区使用的
{
	if (fabs(set_omega - yaw) < 0.3)
	{
		return 0;
	}
	else
	{
		PID_calc(&pid_position_chassis_omega2, yaw, set_omega);
		return pid_position_chassis_omega2.out;
	}
}

fp32 PID_velocity_realize_6020(fp32 set_speed, int i) // can2 6020 speed
{
	PID_calc(&pid_v_chassis_6020[i - 1], motor_info[i - 1].rotor_speed, set_speed);
	return pid_v_chassis_6020[i - 1].out;
}

fp32 PID_velocity_realize_chassis_3508(fp32 set_speed, int i) // can1 3508 speed
{
	PID_calc(&pid_v_chassis_3508[i - 1], motor_can1[i - 1].speed_rpm, set_speed);
	return pid_v_chassis_3508[i - 1].out;
}

fp32 PID_position_6020(fp32 set_pos, int i)
{

	PID_calc_6020_postion(&pid_p_chassis_6020[i - 1], motor_info[i - 1].rotor_angle, set_pos);
	return pid_p_chassis_6020[i - 1].out;
}

fp32 PID_call_6020(fp32 position, int i)
{
	return PID_velocity_realize_6020(PID_position_6020(position, i), i);
}
