#include "chassis_api.h"
#include "imu605_485.h"

extern RC_ctrl_t rc_ctrl;

eChassisAction actChassis = CHASSIS_NORMAL; //底盘默认遥控行走

Chassis_Speed_t absolute_chassis_speed;

Chassis_measure_t absolute_chassis_measure;

fp32 RC_atan2_value = 0;
fp32 yaw_delta;


//fp32 Angle_Helm_Target[3];
fp32 Speed_Motor_Target[3];

fp32 M3508_Target[3];




/**
 * @brief  设定遥控器控制底盘模式
 * @param  void
 * @retval void
 * @attention
 */
void Remote_Control_Chassis_Set_Mode(void)
{
    if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 3) //底盘正常模式
    {
        actChassis = CHASSIS_NORMAL;
    }
    else if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 1) //底盘大陀螺模式   无云台无法实现
    {
        actChassis = CHASSIS_GYROSCOPE;
    }
}


/**
 * @brief  遥控器控制方式
 * @param  chassis_speed 底盘速度
 * @retval void
 * @attention
 */
void Remote_Control_Chassis_Mode(Chassis_Speed_t *chassis_speed)
{
	static bool_t imu_offset_flags = 1;
    /***********************************确定底盘的目标速度*****************************************/
    switch (actChassis)
    {
    case CHASSIS_NORMAL: //正常模式
        chassis_speed->vx = (fp32)rc_ctrl.rc.ch[2]; 
        chassis_speed->vy = (fp32)rc_ctrl.rc.ch[3]; 
        chassis_speed->vw = -(fp32)rc_ctrl.rc.ch[4]; 

        chassis_speed->vx *= 6;
        chassis_speed->vy *= 6;
        chassis_speed->vw *= 1;
        break;

    case CHASSIS_GYROSCOPE: //小陀螺模式
        chassis_speed->vx = (fp32)rc_ctrl.rc.ch[2];
        chassis_speed->vy = (fp32)rc_ctrl.rc.ch[3];
        chassis_speed->vw = -330.0f;

        chassis_speed->vx *= 4;
        chassis_speed->vy *= 4;
        chassis_speed->vw *= 1;
        break;
    default:
        break;
    }
		absolute_chassis_measure.Speed.vx = chassis_speed->vx;
		absolute_chassis_measure.Speed.vy = chassis_speed->vy;
		absolute_chassis_measure.Speed.vw = chassis_speed->vw;
		
		absolute_chassis_measure.Euler.yaw = -imu.Euler.yaw;
		absolute_chassis_measure.Euler.pitch = imu.Euler.pitch;
		absolute_chassis_measure.Euler.roll = imu.Euler.roll;
		
		RC_atan2_value = rad2deg(atan2(absolute_chassis_measure.Speed.vx,absolute_chassis_measure.Speed.vy));
		
		yaw_delta = absolute_chassis_measure.Speed.vw / 660.0f * 0.5f;
		
		absolute_chassis_measure.Euler.yaw_target += yaw_delta;
		
		if(imu_offset_flags == 1)
		{
			absolute_chassis_measure.Euler.yaw_offset = imu.Euler.yaw;
			imu_offset_flags = 0;
		}
		absolute_chassis_measure.Euler.yaw_rel = absolute_chassis_measure.Euler.yaw - absolute_chassis_measure.Euler.yaw_offset;
}

/**
 * @brief  底盘运动解析式计算
 * @param  speed 底盘速度
 * @retval void
* @attention  此函数是全向轮底盘电机的速度解析式
 */
void Chassis_Sports_Calc(Chassis_Speed_t speed)
{
//    Speed_Motor_Target[0] =   speed.vx + speed.vw;
//		Speed_Motor_Target[1] = - speed.vy * sin(deg2rad(60.0f)) - speed.vx * cos(deg2rad(60.0f)) + speed.vw;
//		Speed_Motor_Target[2] =   speed.vy * cos(deg2rad(30.0f)) - speed.vx * sin(deg2rad(30.0f)) + speed.vw;
	
	  Speed_Motor_Target[0] =   speed.vx;
		Speed_Motor_Target[1] = - speed.vy * sin(deg2rad(60.0f)) - speed.vx * cos(deg2rad(60.0f));
		Speed_Motor_Target[2] =   speed.vy * cos(deg2rad(30.0f)) - speed.vx * sin(deg2rad(30.0f));
}




fp32 speed_yaw[3];
/**
 * @brief  底盘电机输出
 * @param  void
 * @retval void
 * @attention
 */
void Chassis_Loop_Out(void)
{
//		speed_yaw[0] = PID_Yaw_realize(0.0f);
//		speed_yaw[1] = PID_Yaw_realize(0.0f);
//		speed_yaw[2] = PID_Yaw_realize(0.0f);
	
		speed_yaw[0] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
		speed_yaw[1] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
		speed_yaw[2] = PID_Yaw_realize(absolute_chassis_measure.Euler.yaw_target);
	
    M3508_Target[0] = PID_velocity_realize_1(Speed_Motor_Target[0] + speed_yaw[0], 1);
    M3508_Target[1] = PID_velocity_realize_1(Speed_Motor_Target[1] + speed_yaw[1], 2);
    M3508_Target[2] = PID_velocity_realize_1(Speed_Motor_Target[2] + speed_yaw[2], 3);

	
	
    CAN1_CMD_1(M3508_Target[0], M3508_Target[1], M3508_Target[2], 0);
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








