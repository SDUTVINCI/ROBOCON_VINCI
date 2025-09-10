#ifndef __CHASSIC_H_
#define __CHASSIC_H_

#ifdef __cplusplus
extern "C"
{
#endif 

#include "main.h"
#include "struct_typedef.h"

#define Radius  0.35//m	
#define WHEEL_Radius 0.07//m
#define PI 3.1415926 
 //true:接收遥控器信息   false：接收上位机控制信息
	
// #define ABS(x) ( (x)>0?(x):-(x) )


typedef struct
{
    bool top;
    bool bottom;
}ROD_STATE;
//底盘结构体
typedef struct
{
	fp32 vx;
	fp32 vy;
	fp32 vomega;
				
}remote_control_speed;
			
typedef struct
{
	int16_t pitch;
	int16_t yaw;
	int16_t roll;
}Euler;

class ROBOT
{
	public:
		ROD_STATE rod_state;
		bool stop,buzzer_flag,bt_auto_fire_flag,fixed_point_flag=0;

		fp32 Launch_Angle;
		fp32 upper_data[5];
		fp32 target_ball_angle;
		uint8_t robot_commend;
		int8_t BT_target_angle;
		fp32 Distance;

		class CHASSIC
		{
			public:
			remote_control_speed Chassic_Move_Data;

			void Omni_Move_Calc();
			void Chassic_Move();
			void Chassic_Mode();
			void Coordinate_Transformation();
			fp32 rad2deg(fp32 rad);


	private:
			fp32 Motor_Speed_Target[4];

		}chassic;
};

//robot类的对象
extern ROBOT robot;


#ifdef __cplusplus
}
#endif 

#endif 