#include "global_position.h"
#include "imu605.h"


extern Chassis_measure_t absolute_chassis_measure;
extern imu605_t imu605;

//fp32 ch1_pre;
/**
 * @brief  获得底盘目标Yaw
 * @param  void
 * @retval void
 * @attention
 */
void Get_Chassis_Yaw_Target(void)
{
//	static fp32 RC_atan2_value = 0;
	static fp32 yaw_delta = 0;
	static fp32 control_ratio = 0.05f;   //速度系数
	
	//单圈控制角度(不启用)
//		RC_atan2_value = rad2deg(atan2(absolute_chassis_measure.Speed.vx,absolute_chassis_measure.Speed.vy));
	//多圈控制角度
		yaw_delta = absolute_chassis_measure.Speed.vw / 660.0f * control_ratio;
		absolute_chassis_measure.Euler.yaw_target += yaw_delta;
//	if(rc_ctrl.rc.ch[1] < 400 && ch1_pre > 400 && ABS(rc_ctrl.rc.ch[0] < 200))
//	{
//		absolute_chassis_measure.Euler.yaw_target += 90.0f;
//	}
//	else if(rc_ctrl.rc.ch[1] > -400 && ch1_pre < -400 && ABS(rc_ctrl.rc.ch[0] < 200))
//	{
//		absolute_chassis_measure.Euler.yaw_target -= 90.0f;
//	}
	
		
//	ch1_pre = rc_ctrl.rc.ch[1];
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

extern fp32 Speed_Yaw[4];
void Yaw_PID_Calc(void)
{
//	Speed_Yaw[0] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
//	Speed_Yaw[0] = ABS(Speed_Yaw[0]);
//	for(int i = 1;i < 4;i++)
//	{Speed_Yaw[i] = Speed_Yaw[0];}
//	if(absolute_chassis_measure.Euler.yaw_target > absolute_chassis_measure.Euler.yaw_total)
//	{
//		Speed_Yaw[1] = -Speed_Yaw[1];
//		Speed_Yaw[2] = -Speed_Yaw[2];
//	}
//	if(absolute_chassis_measure.Euler.yaw_target < absolute_chassis_measure.Euler.yaw_total)
//	{
//		Speed_Yaw[0] = -Speed_Yaw[0];
//		Speed_Yaw[3] = -Speed_Yaw[3];
//	}
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

