#include "pid_user.h"
#include "imu605_485.h"
#include "can_receive.h"

extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];

extern Chassis_measure_t absolute_chassis_measure;

pid_type_def pid_v_1[8],pid_pos_1[8];
pid_type_def pid_v_2[8],pid_pos_2[8];
pid_type_def pid_yaw;

fp32 motor_speed_3508_pid[3] = {10, 0.1, 0};//底盘3508参数
fp32 motor_position_3508_pid[3] = {0.2, 0, 1};
fp32 motor_speed_2006_pid[3] = {8.3,0.1,1};//底盘2006参数
fp32 motor_position_2006_pid[3] = {0.27,0.022,0.3};
fp32 motor_yaw_pid[3] = {15,0.01,0.5};

extern IMU_t imu;

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
	
	PID_init(&pid_yaw,PID_POSITION,motor_yaw_pid,4000,300);
}



fp32 PID_velocity_realize_1(fp32 set_speed,int i)
{
		PID_calc(&pid_v_1[i-1],motor_can1[i-1].speed_rpm , set_speed);
		return pid_v_1[i-1].out;
}

fp32 PID_position_realize_1(fp32 set_pos,int i)
{

		PID_calc(&pid_pos_1[i-1],motor_can1[i-1].total_angle , set_pos);
		return pid_pos_1[i-1].out;

}

fp32 PID_call_1(fp32 position,int i)
{
		return PID_velocity_realize_1(PID_position_realize_1(position,i),i);
}






fp32 PID_velocity_realize_2(fp32 set_speed,int i)
{
		PID_calc(&pid_v_2[i-1],motor_can2[i-1].speed_rpm , set_speed);
		return pid_v_2[i-1].out;
}

fp32 PID_position_realize_2(fp32 set_pos,int i)
{

		PID_calc(&pid_pos_2[i-1],motor_can2[i-1].total_angle , set_pos);
		return pid_pos_2[i-1].out;

}

fp32 PID_call_2(fp32 position,int i)
{
		return PID_velocity_realize_2(PID_position_realize_2(position,i),i);
}



fp32 PID_Yaw_realize(fp32 set_yaw)
{
	int32_t delta_yaw;
	delta_yaw = set_yaw - absolute_chassis_measure.Euler.yaw_rel;
	if(ABS(delta_yaw) <= 2)
	{
		return 0;
	}
	PID_calc(&pid_yaw,absolute_chassis_measure.Euler.yaw_rel,set_yaw);
	return pid_yaw.out;
}
