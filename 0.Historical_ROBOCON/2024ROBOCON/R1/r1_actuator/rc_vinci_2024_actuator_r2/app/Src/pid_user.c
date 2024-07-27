#include "pid_user.h"
#include "can_receive.h"

extern motor_measure_t motor_can1[8];
extern moto_info_t motor_info[MOTOR_MAX_NUM];
extern motor_measure_t motor_can2[8];
pid_type_def pid_v_can1_3508[8], pid_p_can1_3508[8], pid_v_can2_3508[8], pid_p_can2_3508[8], pid_v_can2_2006[8], pid_p_can2_2006[8], pid_v_can1_2006[8], pid_p_can1_2006[8];

float motor_position_3508_pid[3] = {0.2, 0, 1};
float motor_position_3508_arm[3] = {0.4, 0, 0.03}; // 0.5 0 0.1    0.4  0 0.03

float motor_position_3508_pid_sheneg[3] = {0.2, 0, 1};
float motor_position_2006_pid_ARM[3] = {0.3, 0, 0.01};   // 机械抓后臄1�7
float motor_position_2006_pid_gimbal[3] = {0.4, 0, 0.3}; // gimbal

float motor_speed_2006_pid[3] = {3.6, 0, 1.2};	          // 爪子2006参数
float motor_position_2006_pid_HAND[3] = {1.7, 0, 1.2};    // 机械抓pid参数
float motor_speed_3508_pid[3] = {10, 0.1, 0};	          // 爪子
float motor_position_3508_pid_grapball[3] = {0.07, 0, 1}; // 爪子 抓球3508
// CAN1
float motor_position_2006_pid_miao1[3] = {1.6, 0, 0.6}; //
float motor_speed_2006_pid_miao1[3] = {2.8, 0, 1.7};    // 取苗2006参数

float motor_position_2006_pid_miao2[3] = {1.6, 0, 0.9}; //
float motor_speed_2006_pid_miao2[3] = {4, 0, 1};

float motor_position_2006_pid_miao3[3] = {1.4, 0, 0.6}; //
float motor_speed_2006_pid_miao3[3] = {2.8, 0, 1.1};
// CAN2
float motor_position_2006_pid_miao1_CAN2[3] = {1.6, 0, 0.6}; //
float motor_speed_2006_pid_miao1_CAN2[3] = {4, 0, 2.2};      // 取苗2006参数

float motor_position_2006_pid_miao2_CAN2[3] = {1.5, 0, 0.7}; //
float motor_speed_2006_pid_miao2_CAN2[3] = {4, 0, 2.2};

float motor_position_2006_pid_miao3_CAN2[3] = {1.4, 0, 0.6}; //
float motor_speed_2006_pid_miao3_CAN2[3] = {4, 0, 1.1};

float motor_speed_3508_pid_miao[3] = {15, 0.1, 0};     // 爪子
float motor_position_3508_pid_miao[3] = {0.1, 0, 0.1}; // 取苗3508

// float motor_position_3508_pid[3] = {0.2, 0, 1};
// float motor_position_3508_arm[3] = {0.4, 0, 0.03}; // 0.5 0 0.1    0.4  0 0.03

// float motor_position_3508_pid_sheneg[3] = {0.2, 0, 1};
// float motor_position_2006_pid_ARM[3] = {0.3, 0, 0.01};   // 机械抓后臄1�7
// float motor_position_2006_pid_gimbal[3] = {0.4, 0, 0.3}; // gimbal

// float motor_speed_2006_pid[3] = {4, 0, 1};	          // 爪子2006参数
// float motor_position_2006_pid_HAND[3] = {1.7, 0, 1.2};      // 机械抓pid参数
// float motor_speed_3508_pid[3] = {10, 0.1, 0};	          // 爪子
// float motor_position_3508_pid_grapball[3] = {0.07, 0, 1}; // 爪子 抓球3508

// float motor_position_2006_pid_miao1[3] = {1.6, 0, 0.6}; //
// float motor_speed_2006_pid_miao1[3] = {2.8, 0, 1.7};	       // 取苗2006参数

// float motor_position_2006_pid_miao2[3] = {1.6, 0, 0.9}; //
// float motor_speed_2006_pid_miao2[3] = {4, 0, 1};

// float motor_position_2006_pid_miao3[3] = {1.4, 0, 0.6}; //
// float motor_speed_2006_pid_miao3[3] = {2.8, 0, 1.1};

// float motor_speed_3508_pid_miao[3] = {15, 0.1, 0};     // 爪子
// float motor_position_3508_pid_miao[3] = {0.1, 0, 0.1}; // 取苗3508

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

// 底盘电机PID初始匄1�7
void PID_devices_Init(void)
{

	PID_init(&pid_v_can2_2006[4], PID_POSITION, motor_speed_2006_pid, 6000, 1000);
	PID_init(&pid_p_can2_2006[4], PID_POSITION, motor_position_2006_pid_ARM, 2000, 500); // 云台

	PID_init(&pid_v_can1_3508[4], PID_POSITION, motor_speed_3508_pid, 8000, 1000); // zhua爪子
	PID_init(&pid_p_can1_3508[4], PID_POSITION, motor_position_3508_pid_grapball, 1500, 10);

	PID_init(&pid_v_can2_2006[5], PID_POSITION, motor_speed_2006_pid, 8000, 1000); // 爪子
	PID_init(&pid_p_can2_2006[5], PID_POSITION, motor_position_2006_pid_HAND, 2000, 500);

	PID_init(&pid_v_can1_3508[3], PID_POSITION, motor_speed_3508_pid_miao, 10000, 1000); // can1 升降 left
	PID_init(&pid_p_can1_3508[3], PID_POSITION, motor_position_3508_pid_miao, 10000, 10);

	PID_init(&pid_v_can2_3508[3], PID_POSITION, motor_speed_3508_pid_miao, 10000, 1000); // can1 升降 left
	PID_init(&pid_p_can2_3508[3], PID_POSITION, motor_position_3508_pid_miao, 10000, 10);

	PID_init(&pid_p_can1_2006[0], PID_POSITION, motor_position_2006_pid_miao1, 2000, 20);
	PID_init(&pid_v_can1_2006[0], PID_POSITION, motor_speed_2006_pid_miao1, 4300, 20);

	PID_init(&pid_p_can1_2006[1], PID_POSITION, motor_position_2006_pid_miao2, 2000, 20);
	PID_init(&pid_v_can1_2006[1], PID_POSITION, motor_speed_2006_pid_miao2, 4400, 20);

	PID_init(&pid_p_can1_2006[2], PID_POSITION, motor_position_2006_pid_miao3, 2000, 20);
	PID_init(&pid_v_can1_2006[2], PID_POSITION, motor_speed_2006_pid_miao3, 4300, 20);

	PID_init(&pid_p_can2_2006[0], PID_POSITION, motor_position_2006_pid_miao1_CAN2, 2000, 20);
	PID_init(&pid_v_can2_2006[0], PID_POSITION, motor_speed_2006_pid_miao1_CAN2, 4500, 20);

	PID_init(&pid_p_can2_2006[1], PID_POSITION, motor_position_2006_pid_miao2_CAN2, 2000, 20);
	PID_init(&pid_v_can2_2006[1], PID_POSITION, motor_speed_2006_pid_miao2_CAN2, 4300, 20);

	PID_init(&pid_p_can2_2006[2], PID_POSITION, motor_position_2006_pid_miao3_CAN2, 2000, 20);
	PID_init(&pid_v_can2_2006[2], PID_POSITION, motor_speed_2006_pid_miao3_CAN2, 4500, 20);
}

fp32 PID_velocity_realize_chassis_3508(fp32 set_speed, int i) // can1 3508 speed
{
	PID_calc(&pid_v_can1_3508[i - 1], motor_can1[i - 1].speed_rpm, set_speed);
	return pid_v_can1_3508[i - 1].out;
}

fp32 PID_velocity_realize_chassis_2006(fp32 set_speed, int i) // can1 2006 chalss 456
{
	PID_calc(&pid_v_can1_2006[i - 1], motor_can1[i - 1].speed_rpm, set_speed);
	return pid_v_can1_2006[i - 1].out;
}
fp32 PID_position_realize_3508(fp32 set_pos, int i)
{

	PID_calc(&pid_p_can1_3508[i - 1], motor_can1[i - 1].total_angle, set_pos);
	return pid_p_can1_3508[i - 1].out;
}

fp32 PID_position_realize_chassis_2006(fp32 set_pos, int i) // can1 2006 chalss
{

	PID_calc(&pid_p_can1_2006[i - 1], motor_can1[i - 1].total_angle, set_pos);
	return pid_p_can1_2006[i - 1].out;
}

fp32 PID_call_2006(fp32 position, int i) // can1 2006 chalss
{
	return PID_velocity_realize_chassis_2006(PID_position_realize_chassis_2006(position, i), i);
}

fp32 PID_call_3508(fp32 position, int i) // can1 2006 chalss
{
	return PID_velocity_realize_chassis_3508(PID_position_realize_3508(position, i), i);
}

fp32 PID_velocity_realize_2_3508(fp32 set_speed, int i)
{
	PID_calc(&pid_v_can2_3508[i - 1], motor_can2[i - 1].speed_rpm, set_speed);
	return pid_v_can2_3508[i - 1].out;
}

fp32 PID_position_realize_2_3508(fp32 set_pos, int i)
{

	PID_calc(&pid_p_can2_3508[i - 1], motor_can2[i - 1].total_angle, set_pos);
	return pid_p_can2_3508[i - 1].out;
}

fp32 PID_call_2_3508(fp32 position, int i)
{
	return PID_velocity_realize_2_3508(PID_position_realize_2_3508(position, i), i);
}

fp32 PID_velocity_realize_2_2006(fp32 set_speed, int i)
{
	PID_calc(&pid_v_can2_2006[i - 1], motor_can2[i - 1].speed_rpm, set_speed);
	return pid_v_can2_2006[i - 1].out;
}

fp32 PID_position_realize_2_2006(fp32 set_pos, int i)
{

	PID_calc(&pid_p_can2_2006[i - 1], motor_can2[i - 1].total_angle, set_pos);
	return pid_p_can2_2006[i - 1].out;
}

fp32 PID_call_2_2006(fp32 position, int i)
{
	return PID_velocity_realize_2_2006(PID_position_realize_2_2006(position, i), i);
}
