#include "pid_user.h"
#include "can_receive.h"
#include "imu605.h"
extern fp32 Vomega;
extern motor_measure_t motor_can1[8];
extern moto_info_t motor_info[MOTOR_MAX_NUM];

pid_type_def pid_v_chassis_3508[3]
, pid_v_chassis_6020[3]
,pid_p_chassis_6020[3]

,pid_position_chassis_omega
,pid_position_chassis_y
,pid_position_chassis_x
,pid_position_omega_speed
,pid_position_L1s_X2
,pid_position_L1s_Y2
,pid_position_L1s_X1
,pid_position_L1s_Y1;//  0:  x方向   1：y方向


float motor_position_chassis_L1S[3] = {10, 0, 0};

float motor_position_omega_speed[3]={0.03,0.0,0};//


float motor_speed_3508_pid[3] = {15, 0, 0}; // 底盘3508参数
float motor_position_3508_pid[3] = {0.2, 0, 1};
float motor_speed_2006_pid[3] = {8.3, 0.1, 1}; // 底盘2006参数
float motor_position_2006_pid[3] = {0.27, 0.022, 0.7};
float motor_speed_6020_pid[3] = {40, 3, 0};
float motor_position_6020_pid[3]={1,0,4};
float motor_position_chassis[3]={10,0,0.02};
float motor_position_chassis_omega[3]={19,0,10};//21 0 2.0      21.6,0,2.14
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
		PID_init(&pid_v_chassis_3508[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000); // 3508  speed   
		PID_init(&pid_v_chassis_6020[i], PID_POSITION, motor_speed_6020_pid, 30000, 30000); // 6020 speed
		PID_init(&pid_p_chassis_6020[i], PID_POSITION, motor_position_6020_pid, 20000, 20000); // 6020 position
		
	}
//	PID_init(&pid_position_chassis_x, PID_POSITION, motor_position_chassis, 2300, 1200);
//	PID_init(&pid_position_chassis_y, PID_POSITION, motor_position_chassis, 2300, 1200);
//	PID_init(&pid_position_chassis_omega, PID_POSITION, motor_position_chassis_omega,10, 3);
//	
//	
	
	
	
	
	PID_init(&pid_position_L1s_X1, PID_POSITION, motor_position_chassis_L1S, 2300, 300); // 激光测距定位
	PID_init(&pid_position_L1s_Y1, PID_POSITION, motor_position_chassis_L1S, 2300, 300);
	PID_init(&pid_position_L1s_X2, PID_POSITION, motor_position_chassis_L1S, 2300, 300); 
	PID_init(&pid_position_L1s_Y2, PID_POSITION, motor_position_chassis_L1S, 2300, 300);	
	
		PID_init(&pid_position_omega_speed, PID_POSITION, motor_position_omega_speed, 10, 5);
//	
	PID_init(&pid_position_chassis_x, PID_POSITION, motor_position_chassis, 330, 330);
	PID_init(&pid_position_chassis_y, PID_POSITION, motor_position_chassis, 330, 330);
	PID_init(&pid_position_chassis_omega, PID_POSITION, motor_position_chassis_omega, 500,2);
}
// fp32 PID_position_chassis_x(fp32 set_position)//position
//{
//		PID_calc(&pid_position_chassis_x ,ops9.Position.pos_x , set_position);
//		return pid_position_chassis_x.out;
//}
// fp32 PID_position_chassis_y(fp32 set_position )//position
//{
//		PID_calc(&pid_position_chassis_y,ops9.Position.pos_y, set_position);
//		return pid_position_chassis_y.out;
//}

extern fp32 yaw;
 fp32 PID_position_chassis_omega(fp32 set_omega )//position
{
		PID_calc(&pid_position_chassis_omega,yaw, set_omega);
		return pid_position_chassis_omega.out;
}



 fp32 PID_position_omega_speed(fp32 set_speed )//position
{
		PID_calc(&pid_position_omega_speed,Vomega, set_speed);
		return pid_position_omega_speed.out;
}

 fp32 PID_call_omega(fp32 position)
{
		return PID_position_omega_speed(PID_position_chassis_omega(position));
}


 fp32 PID_velocity_realize_6020(fp32 set_speed,int i)//can2 6020 speed
{
		PID_calc(&pid_v_chassis_6020[i-1],motor_info[i-1].rotor_speed , set_speed);
		return pid_v_chassis_6020[i-1].out;
}

fp32 PID_velocity_realize_chassis_3508(fp32 set_speed, int i) // can1 3508 speed
{
	PID_calc(&pid_v_chassis_3508[i - 1], motor_can1[i - 1].speed_rpm, set_speed);
	return pid_v_chassis_3508[i - 1].out;
}

fp32 PID_position_6020(fp32 set_pos,int i)
{

		PID_calc_6020_postion(&pid_p_chassis_6020[i-1],motor_info[i-1].rotor_angle , set_pos);
		return pid_p_chassis_6020[i-1].out;

}

 fp32 PID_call_6020(fp32 position,int i)
{
		return PID_velocity_realize_6020(PID_position_6020(position,i),i);
}





#include <stdlib.h>

extern unsigned int gdistance_Y2  ;
extern unsigned int gdistance_X2  ;
extern unsigned int gdistance_Y1  ;
extern unsigned int gdistance_X1  ;


fp32 PID_position_L1S_Y1(int set_position) // position
{
	//int tttt = set_position - CalcKalmanFilter(&Kalman_L1sY1, gdistance_Y) * cos((90 + yaw) * 3.14 / 360);
	//int tttt = set_position - gdistance_Y1;


		PID_calc(&pid_position_L1s_Y1, gdistance_Y1 , set_position);
		return pid_position_L1s_Y1.out;
	
}




fp32 PID_position_L1S_X1(int set_position) // position
{

		PID_calc(&pid_position_L1s_X1, gdistance_X1 , set_position);
		return pid_position_L1s_X1.out;
	
}

fp32 PID_position_L1S_X2(int set_position) // position
{

		PID_calc(&pid_position_L1s_X2, gdistance_X2 , set_position);
		return pid_position_L1s_X2.out;
	
}


fp32 PID_position_L1S_Y2(int set_position) // position
{

		PID_calc(&pid_position_L1s_Y2, gdistance_Y2 , set_position);
		return pid_position_L1s_Y2.out;
	
}