#include "global_position.h"
#include "hwt101ct_232.h"
#include "helm_wheel.h"

extern Chassis_measure_t absolute_chassis_measure;
extern HWT101_t hwt101ct;
extern fp32 speed_yaw[4];
extern fp32 position_yaw[4];

extern fp32 speed_pos_x[4];
extern fp32 speed_pos_y[4];

extern fp32 position_pos_x[4];
extern fp32 position_pos_y[4];

extern fp32 Angle_Helm_Target[4];
extern fp32 Speed_Motor_Target[4];
extern fp32 direction_coefficient;

extern fp32 M3508_Target[4];
extern fp32 M2006_Target[4];
extern RC_ctrl_t rc_ctrl;
bool_t GP_SWITCH_STATUS = 0;

extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];

/***************************************惯性计矫正********************************************/
fp32 RC_atan2_value;
/**
 * @brief  获得底盘目标Yaw
 * @param  void
 * @retval void
 * @attention
 */
void Get_Chassis_Yaw_Target(void)
{
	static fp32 yaw_delta = 0;
	static fp32 control_ratio = 0.4f;   //速度系数
	
	//单圈控制角度(不启用)
		RC_atan2_value = rad2deg(atan2(absolute_chassis_measure.Speed.vx,absolute_chassis_measure.Speed.vy));
	//多圈控制角度
		yaw_delta = absolute_chassis_measure.Speed.vw / 660.0f * control_ratio;
		absolute_chassis_measure.Euler.yaw_target += yaw_delta;
}


/**
 * @brief  获得底盘修正Yaw和修正后的相对Yaw
 * @param  void
 * @retval void
 * @attention
 */
void Get_Chassis_Yaw_Offset(void)
{
		static bool_t imu_offset_flags = 1;
		if(imu_offset_flags == 1)
		{
			absolute_chassis_measure.Euler.yaw_offset = absolute_chassis_measure.Euler.yaw;
			imu_offset_flags = 0;
		}
		absolute_chassis_measure.Euler.yaw_last_angle = absolute_chassis_measure.Euler.yaw_rel;
		absolute_chassis_measure.Euler.yaw_rel = absolute_chassis_measure.Euler.yaw - absolute_chassis_measure.Euler.yaw_offset;
}


/**
 * @brief  获得底盘总Yaw
 * @param  void
 * @retval void
 * @attention  可以实现多圈
 */
void Get_Chassis_Total_Yaw(void)
{
		if(absolute_chassis_measure.Euler.yaw_rel - absolute_chassis_measure.Euler.yaw_last_angle < -300.0f)
		{
			absolute_chassis_measure.Euler.yaw_round_cnt++;
		}
		else if(absolute_chassis_measure.Euler.yaw_rel - absolute_chassis_measure.Euler.yaw_last_angle > 300.0f)
		{
			absolute_chassis_measure.Euler.yaw_round_cnt--;
		}
		absolute_chassis_measure.Euler.yaw_total = 360.0f * absolute_chassis_measure.Euler.yaw_round_cnt + absolute_chassis_measure.Euler.yaw_rel;
}


void Yaw_PID_Calc(fp32 atan2_angle)
{
			for (int i = 0; i < 4; i++)
		{
			speed_yaw[i] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
			// position_yaw[i] = PID_Yaw_position_realize(absolute_chassis_measure.Euler.yaw_target);
			speed_yaw[i] = ABS(speed_yaw[i]);
			speed_yaw[i] *= 3.0f;
		}
		
		if((0.0f <= ABS(atan2_angle) && ABS(atan2_angle) <= 45.0f) || (135.0f <= ABS(atan2_angle) && ABS(atan2_angle) <= 180.0f))
		{
			if(absolute_chassis_measure.Euler.yaw_total > absolute_chassis_measure.Euler.yaw_target)//逆时针
			{
				speed_yaw[1] *= -1;
				speed_yaw[2] *= -1;
			}
			else if(absolute_chassis_measure.Euler.yaw_total <= absolute_chassis_measure.Euler.yaw_target)//顺时针
			{
				speed_yaw[0] *= -1;
				speed_yaw[3] *= -1;
			}
		}
		else if(45.0f < atan2_angle && atan2_angle < 135.0f)   //右拨
		{
			if(absolute_chassis_measure.Euler.yaw_total > absolute_chassis_measure.Euler.yaw_target)  //逆时针
			{
				speed_yaw[0] *= -1;
				speed_yaw[1] *= -1;
			}
			else if(absolute_chassis_measure.Euler.yaw_total <= absolute_chassis_measure.Euler.yaw_target)   //顺时针
			{
				speed_yaw[2] *= -1;
				speed_yaw[3] *= -1;
			}
		}
		else if(-135.0f < atan2_angle && atan2_angle < -45.0f)   //左拨
		{
			if(absolute_chassis_measure.Euler.yaw_total > absolute_chassis_measure.Euler.yaw_target)  //逆时针
			{
				speed_yaw[2] *= -1;
				speed_yaw[3] *= -1;
			}
			else if(absolute_chassis_measure.Euler.yaw_total <= absolute_chassis_measure.Euler.yaw_target)   //顺时针
			{
				speed_yaw[0] *= -1;
				speed_yaw[1] *= -1;
			}
		}
		
		
			if (ABS(rc_ctrl.rc.ch[2]) <= 250 && ABS(rc_ctrl.rc.ch[3]) <= 250)
			{
				for(int i = 0;i < 4;i++)
				{
					speed_yaw[i] = 0;
				}
			}
			//修复bug:失控转大圈(非最优解，矢量为最优解)
			if((45.0f < atan2_angle && atan2_angle < 135.0f) || (-135.0f < atan2_angle && atan2_angle < -45.0f))
			{
				for(int i = 0;i < 4;i++)
				{
					speed_yaw[i] *= -direction_coefficient;
				}
			}
}

/***************************************码盘全局定位********************************************/


void OPS9_Reset_Origin(UART_HandleTypeDef *huart)
{
//	HAL_Delay(15);
	osDelay(15);
	Update_x(huart,0.0);
//	HAL_Delay(15);
	osDelay(15);
	Update_y(huart,0.0);
//	HAL_Delay(15);
	osDelay(15);
}

void OPS9_Set_Coordinate(fp32 X_Target,fp32 Y_Target,fp32 Yaw_Target)
{
	absolute_chassis_measure.Position.Pos_X_Target = X_Target;
	absolute_chassis_measure.Position.Pos_Y_Target = Y_Target;
	absolute_chassis_measure.Euler.yaw_target = Yaw_Target;
}

fp32 OPS9_Angle_Target;
fp32 OSP9_Angle_Target_Calc(void)
{
	OPS9_Angle_Target = rad2deg(atan2(absolute_chassis_measure.Position.Pos_X_Target,absolute_chassis_measure.Position.Pos_Y_Target));
	return OPS9_Angle_Target;
}

extern fp32 res;
extern fp32 prev_res;
extern fp32 res1;
extern fp32 round_cnt;
extern fp32 round1;
extern fp32 round2;
extern fp32 direction_coefficient;


void OPS9_GP_Stage1(void)
{
//	if(rc_ctrl.rc.ch[3] > 500)
//	{
//		OPS9_Set_Coordinate(100.0,0.0,0.0);
//	}
//	else if(rc_ctrl.rc.ch[3] < -500)
//	{
//		OPS9_Set_Coordinate(0.0,0.0,0.0);
//	}
//	
//	res = -OSP9_Angle_Target_Calc();
//	for(int i = 0;i < 4;i++)
//	{
//		Speed_Motor_Target[i] = 0;
//	}
//	New_Helm_Angle_Calc();
//	Loop_PID_CMD();
}


void New_Helm_Angle_Calc(void)
{
	res1 = calc_min_angle(res, prev_res);							 //就近转圈的目标角度

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
}

fp32 target_temp[4];
void Loop_PID_CMD(void)
{
	for(int i = 0;i < 4;i++)
	{
		
//		speed_yaw[i] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
		Yaw_PID_Calc(prev_res);
		position_pos_x[i] = PID_Pos_X_realize(absolute_chassis_measure.Position.Pos_X_Target);
		position_pos_y[i] = PID_Pos_Y_realize(absolute_chassis_measure.Position.Pos_Y_Target);
		M2006_Target[i] = PID_call_2(Angle_Helm_Target[i],i+1);
		if(ABS(Angle_Helm_Target[i] - motor_can2[i].total_angle) <= 4096)
		{
			target_temp[i] = position_pos_x[i] + position_pos_y[i];
		}
		else
		{
			target_temp[i] = 0;
		}
		target_temp[i] *= -3;
		M3508_Target[i] = PID_velocity_realize_1((target_temp[i] + speed_yaw[i]),i+1);
	}
	CAN1_CMD_1(M3508_Target[0],M3508_Target[1],M3508_Target[2],M3508_Target[3]);
	CAN2_CMD_1(M2006_Target[0],M2006_Target[1],M2006_Target[2],M2006_Target[3]);
	
}




/**
 * @brief  角度转换为弧度
 * @param  deg 角度值
 * @retval rad 弧度值
 * @attention
 */
fp32 deg2rad(fp32 deg)
{
    fp32 rad;
    rad = deg * (PI / 180.0f);
    return rad;
}

/**
 * @brief  弧度转换为角度
 * @param  rad 弧度值
 * @retval deg 角度值
 * @attention
 */
fp32 rad2deg(fp32 rad)
{
    fp32 deg;
    deg = rad * (180.0f / PI);
    return deg;
}

