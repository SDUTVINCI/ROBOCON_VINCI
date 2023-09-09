#include "chassis_api.h"
#include "global_position.h"
#include "helm_wheel.h"
#include "hwt101ct_232.h"

extern RC_ctrl_t rc_ctrl;
extern HWT101_t hwt101ct;
extern fp32 RC_atan2_value;
extern bool_t GP_SWITCH_STATUS;
eChassisAction actChassis = CHASSIS_NORMAL; //底盘默认遥控行走

Chassis_Speed_t absolute_chassis_speed;

Chassis_measure_t absolute_chassis_measure;

fp32 Angle_Helm_Target[4];
fp32 Speed_Motor_Target[4];

fp32 M3508_Target[4];
fp32 M2006_Target[4];

extern ops9_t ops9;
bool_t actChassis_pre = 1;
/**
 * @brief  设定遥控器控制底盘模式
 * @param  void
 * @retval void
 * @attention
 */
void Remote_Control_Chassis_Set_Mode(void)
{
	actChassis_pre = actChassis;
	if (rc_ctrl.rc.s[0] == 3) //底盘正常模式
	{
		actChassis = CHASSIS_NORMAL;
	}
	if (rc_ctrl.rc.s[0] == 1) //底盘正常模式
	{
		actChassis = CHASSIS_NORMAL;
	}
//	if (rc_ctrl.rc.s[0] == 1)
//	{
//		actChassis = CHASSIS_GLOBAL_POSITION;
//	}
//	else if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 1) //底盘大陀螺模式   无云台无法实现
//	{
//		actChassis = CHASSIS_GYROSCOPE;
//	}
}

/**
 * @brief  遥控器控制方式
 * @param  chassis_speed 底盘速度
 * @retval void
 * @attention
 */
void Remote_Control_Chassis_Mode(Chassis_Speed_t *chassis_speed)
{
	
	/***********************************确定底盘的目标速度*****************************************/
	switch (actChassis)
	{
	case CHASSIS_NORMAL: //正常模式
		chassis_speed->vx = (fp32)rc_ctrl.rc.ch[2];
		chassis_speed->vy = (fp32)rc_ctrl.rc.ch[3];
		chassis_speed->vw = -(fp32)rc_ctrl.rc.ch[4];

		chassis_speed->vx *= 0.2f;
		chassis_speed->vy *= 0.2f;
		chassis_speed->vw *= 0.2f;
	  GP_SWITCH_STATUS = 0;
		break;

	case CHASSIS_GYROSCOPE: //小陀螺模式
		chassis_speed->vx = (fp32)rc_ctrl.rc.ch[2];
		chassis_speed->vy = (fp32)rc_ctrl.rc.ch[3];
		chassis_speed->vw = -330.0f;

		chassis_speed->vx *= 4;
		chassis_speed->vy *= 4;
		chassis_speed->vw *= 1;
	  GP_SWITCH_STATUS = 0;
		break;
	
	case CHASSIS_GLOBAL_POSITION://全局定位模式
//		absolute_chassis_measure.Position.Pos_X_Target = 0.0f;
//		absolute_chassis_measure.Position.Pos_Y_Target = 200.0f;
		  GP_SWITCH_STATUS = 1;
	if(actChassis == CHASSIS_GLOBAL_POSITION && actChassis_pre == CHASSIS_NORMAL)
	{
		OPS9_Reset_Origin(&huart2);
	}
		break;
	default:
		break;
	}
}

void Robot_Pose_Analysis(void)
{
	absolute_chassis_measure.Speed.vx = absolute_chassis_speed.vx;
	absolute_chassis_measure.Speed.vy = absolute_chassis_speed.vy;
	absolute_chassis_measure.Speed.vw = absolute_chassis_speed.vw;

	absolute_chassis_measure.Euler.yaw = -hwt101ct.Euler.yaw;
	//		absolute_chassis_measure.Euler.yaw = - ops9.Euler.yaw;
	//		absolute_chassis_measure.Euler.pitch = hwt101ct.Euler.pitch;
	//		absolute_chassis_measure.Euler.roll = hwt101ct.Euler.roll;

	absolute_chassis_measure.Position.Pos_X = ops9.Position.pos_x;
	absolute_chassis_measure.Position.Pos_Y = ops9.Position.pos_y;

	//获取目标Yaw
	Get_Chassis_Yaw_Target();

	//获取修正后的Yaw
	Get_Chassis_Yaw_Offset();

	//获取累计Yaw
	Get_Chassis_Total_Yaw();
}

fp32 res = 0.00f, prev_res = 0.00f, res1 = 0.00f;
fp32 round_cnt = 0.00f;
fp32 round1 = 0.00f, round2 = 0.00f;
fp32 direction_coefficient = 1.00f;
bool_t spin_flags = 0;

fp32 speed_buffer[2],angle_buffer[2];

/**
 * @brief  底盘运动解析式计算
 * @param  speed 底盘速度
 * @retval void
 * @attention  此函数是舵轮底盘电机的速度解析式
 */
void Chassis_Sports_Calc(Chassis_Speed_t speed)
{
	res = calc_angle_helm_wheel(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]); //计算遥控器遥杆角度
	
	res1 = calc_min_angle(res, prev_res);							 //就近转圈的目标角度
//	res1 = 0;

	round_cnt = calc_motor_round_cnt((res + res1), prev_res); //计算圈数
	round1 = fabs(prev_res - 1);
	round2 = fabs(res - prev_res);
	if (round1 >= round2)
	{
		res1 = 0.00f;
		direction_coefficient = 1.00f;
	}
	if (round1 < round2)
	{
		res = 0.00f;
		direction_coefficient = -1.00f;
	}
	prev_res = res + res1;																				 //记录上一次的角度数据
	Angle_Helm_Target[0] = 67.00f / 20.00f * (36.00f * 8192.00f * (round_cnt + (res + res1) / 360.00f)); //将数据转换成轮子转动的角度数据
	Angle_Helm_Target[1] = 67.00f / 20.00f * (36.00f * 8192.00f * (round_cnt + (res + res1) / 360.00f));
	Angle_Helm_Target[2] = 67.00f / 20.00f * (36.00f * 8192.00f * (round_cnt + (res + res1) / 360.00f));
	Angle_Helm_Target[3] = 67.00f / 20.00f * (36.00f * 8192.00f * (round_cnt + (res + res1) / 360.00f));

	if (ABS(rc_ctrl.rc.ch[2]) > 250 || ABS(rc_ctrl.rc.ch[3]) > 250)
	{
//		for(int i = 0;i < 4;i++)
//		{
//			Speed_Motor_Target[i] = -sqrt((rc_ctrl.rc.ch[2] - 250) * (rc_ctrl.rc.ch[2] - 250) + (rc_ctrl.rc.ch[3] - 250) * (rc_ctrl.rc.ch[3] - 250)) * direction_coefficient;
//		}
//		Speed_Motor_Target[0] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3]));
//		Speed_Motor_Target[1] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3]) - rc_ctrl.rc.ch[3] / 98.636f);
//		Speed_Motor_Target[2] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3]) - rc_ctrl.rc.ch[3] / 98.636f);
//		Speed_Motor_Target[3] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3]));
		
			Speed_Motor_Target[0] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] * 4.3f + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3] * 0.8f));
			Speed_Motor_Target[1] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] * 4.3f + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3] * 0.8f));
			Speed_Motor_Target[2] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] * 4.3f + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3] * 0.8f));
			Speed_Motor_Target[3] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] * 4.3f + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3] * 0.8f));
			
		if(ABS(rc_ctrl.rc.ch[3]) >= 645)
		{
			Speed_Motor_Target[0] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] * 4.3f + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3] * 2.8f));
			Speed_Motor_Target[1] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] * 4.3f + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3] * 2.8f));
			Speed_Motor_Target[2] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] * 4.3f + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3] * 2.8f));
			Speed_Motor_Target[3] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] * 4.3f + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3] * 2.8f));
		}
		
	 if ((420 >= ABS(rc_ctrl.rc.ch[2]) && ABS(rc_ctrl.rc.ch[2]) > 250) || (420 >= ABS(rc_ctrl.rc.ch[3]) && ABS(rc_ctrl.rc.ch[3]) > 250))
	 {
		Speed_Motor_Target[0] *= 0.8f;
		Speed_Motor_Target[1] *= 0.8f;
		Speed_Motor_Target[2] *= 0.8f;
		Speed_Motor_Target[3] *= 0.8f;
	 }
	 	if (ABS(rc_ctrl.rc.ch[2]) > 420 || ABS(rc_ctrl.rc.ch[3]) > 250)
	 {
		Speed_Motor_Target[0] *= 4.3f;
		Speed_Motor_Target[1] *= 4.3f;
		Speed_Motor_Target[2] *= 4.3f;
		Speed_Motor_Target[3] *= 4.3f;
	 }
	}
	if (ABS(rc_ctrl.rc.ch[2]) <= 250 && ABS(rc_ctrl.rc.ch[3]) <= 250)
	{
		Speed_Motor_Target[0] = 0;
		Speed_Motor_Target[1] = 0;
		Speed_Motor_Target[2] = 0;
		Speed_Motor_Target[3] = 0;
	}

	if (ABS(rc_ctrl.rc.ch[4]) != 0)
	{
		spin_flags = 1;
		if (ABS(rc_ctrl.rc.ch[4]) >= 200)
		{
			Angle_Helm_Target[0] = -123494.400f + 67.00f / 20.00f * round_cnt * 8192.00f * 36.00f;
			Angle_Helm_Target[1] = 123494.400f + 67.00f / 20.00f * round_cnt * 8192.00f * 36.00f;
			Angle_Helm_Target[2] = -123494.400f + 67.00f / 20.00f * round_cnt * 8192.00f * 36.00f;
			Angle_Helm_Target[3] = 123494.400f + 67.00f / 20.00f * round_cnt * 8192.00f * 36.00f;
		}
		if (ABS(rc_ctrl.rc.ch[4]) >= 330)
		{
			Speed_Motor_Target[0] = +0.707106f * rc_ctrl.rc.ch[4]; // 原地旋转
			Speed_Motor_Target[1] = -0.707106f * rc_ctrl.rc.ch[4];
			Speed_Motor_Target[2] = -0.707106f * rc_ctrl.rc.ch[4];
			Speed_Motor_Target[3] = +0.707106f * rc_ctrl.rc.ch[4];

	 if ((350 >= ABS(rc_ctrl.rc.ch[4]) && ABS(rc_ctrl.rc.ch[4]) > 250) || (350 >= ABS(rc_ctrl.rc.ch[4]) && ABS(rc_ctrl.rc.ch[4]) > 250))
	 {
		Speed_Motor_Target[0] *= 0.5f;
		Speed_Motor_Target[1] *= 0.5f;
		Speed_Motor_Target[2] *= 0.5f;
		Speed_Motor_Target[3] *= 0.5f;
	 }
	 	if (ABS(rc_ctrl.rc.ch[4]) > 350 || ABS(rc_ctrl.rc.ch[4]) > 250)
	 {
		Speed_Motor_Target[0] *= 4.3f;
		Speed_Motor_Target[1] *= 4.3f;
		Speed_Motor_Target[2] *= 4.3f;
		Speed_Motor_Target[3] *= 4.3f;
	 }
		}
		absolute_chassis_measure.Euler.yaw_target = absolute_chassis_measure.Euler.yaw_total;
	}
	else
	{
		spin_flags = 0;
	}
	
		if(rc_ctrl.rc.s[0] == 3 || rc_ctrl.rc.s[0] == 1)
		{
			speed_buffer[0] = Speed_Motor_Target[0];
			speed_buffer[1] = Speed_Motor_Target[1];
			Speed_Motor_Target[0] = Speed_Motor_Target[2];
			Speed_Motor_Target[1] = Speed_Motor_Target[3];
			Speed_Motor_Target[2] = speed_buffer[0];
			Speed_Motor_Target[3] = speed_buffer[1];
			
			for(int i = 0;i < 4;i++)
			{
				Speed_Motor_Target[i] *= -1;
			}	
			
			angle_buffer[0] = Angle_Helm_Target[0];
			angle_buffer[1] = Angle_Helm_Target[1];
			Angle_Helm_Target[0] = Angle_Helm_Target[2];
			Angle_Helm_Target[1] = Angle_Helm_Target[3];
			Angle_Helm_Target[2] = angle_buffer[0];
			Angle_Helm_Target[3] = angle_buffer[1];
		}

}

fp32 speed_yaw[4];
fp32 position_yaw[4];

fp32 speed_pos_x[4];
fp32 speed_pos_y[4];

fp32 position_pos_x[4];
fp32 position_pos_y[4];



fp32 speed_target_final[4];
/**
 * @brief  底盘电机输出
 * @param  void
 * @retval void
 * @attention
 */
void Chassis_Loop_Out(void)
{
	
	if(spin_flags == 0)
	{
		Yaw_PID_Calc(RC_atan2_value);

		for (int i = 0; i < 4; i++)
		{
			speed_target_final[i] = Speed_Motor_Target[i] + speed_yaw[i];
			M3508_Target[i] = PID_velocity_realize_1(speed_target_final[i], i + 1);
			M2006_Target[i] = PID_call_2(Angle_Helm_Target[i], i + 1);
		}
	}
	else if (spin_flags == 1)
	{
		for (int i = 0; i < 4; i++)
		{
			speed_target_final[i] = Speed_Motor_Target[i];
			M3508_Target[i] = PID_velocity_realize_1(speed_target_final[i], i + 1);
			M2006_Target[i] = PID_call_2(Angle_Helm_Target[i], i + 1);
		}
	}
	
	CAN1_CMD_1(M3508_Target[0], M3508_Target[1], M3508_Target[2], M3508_Target[3]);
	CAN2_CMD_1(M2006_Target[0], M2006_Target[1], M2006_Target[2], M2006_Target[3]);
}
