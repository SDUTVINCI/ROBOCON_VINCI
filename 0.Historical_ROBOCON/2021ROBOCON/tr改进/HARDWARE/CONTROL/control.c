#include "control.h"
#include "pid_user.h"
#include "math.h"
#include "main.h"
#include "usart.h"

extern pid_type_def pid_x;
extern pid_type_def pid_y;

float a_1[3] = {-6942.98f,558.37,0.0f};//-6942.98  558.37
float a_2[3] = {-2865.18,19.366,0.0f};//-2865.18 19.366
float a_3[3] = {-7062.9,110.44,0.0f};//-7062.9 110.44
float a_4[3] = {-3448.266f, 445.13f,0.0f};//-3448.26  445.13
float a_5[3] = {-7809.015f, 152.3f,0.0f};//-7809.015 152.3





float x_err = 5.1f;
float y_err = 5.1f;
float yaw_err = 0.0001f;

extern float len_back,len_right,yaw;






//uint8_t Jugement_position(float x, float y, float yaw_)
//{
//	if (fabs(len_right - x)<x_err && fabs(len_back - y)<y_err && fabs(yaw_ - yaw)<yaw_err)
//	{
//		return 1;
//	}
//	else 
//		return 0;
//}



extern float zangle;
extern float pos_x;
extern float pos_y;


uint8_t Jugement_position(float x, float y, float yaw_)
{
	if (fabs(pos_x - x)<x_err  && fabs(yaw_ - zangle)<yaw_err  && fabs(pos_y - y)<y_err)
	{
		return 1;
	}
	else 
		return 0;
}




void Car_position_control(float x, float y, float yaw)
{
	    v_x = PID_X_POS_realize(x)*cosf(yaw/180.0f*3.14159f)+PID_Y_POS_realize(y)*sinf(yaw/180.0f*3.14159f);
			v_y = -PID_X_POS_realize(x)*sinf(yaw/180.0f*3.14159f)+PID_Y_POS_realize(y)*cosf(yaw/180.0f*3.14159f);
			v_yaw = PID_YAW_POS_realize(yaw);
			Speed_Motor_Target_1 =  0.707106f * v_x + 0.707106f * v_y - v_yaw;
			Speed_Motor_Target_2 =  0.707106f * v_x - 0.707106f * v_y - v_yaw;
			Speed_Motor_Target_3 = -0.707106f * v_x - 0.707106f * v_y - v_yaw;
			Speed_Motor_Target_4 = -0.707106f * v_x + 0.707106f * v_y - v_yaw;
			CAN_CMD_can1_1(PID_velocity_realize_1(Speed_Motor_Target_1,1),
											PID_velocity_realize_1(Speed_Motor_Target_2,2),
												PID_velocity_realize_1(Speed_Motor_Target_3,3),
													PID_velocity_realize_1(Speed_Motor_Target_4,4));
	   
			
}


float aaaa = 10.0f;

//void Set_mapan_position(void)
//{
//	if (len_right>-120000 && len_back<60000)
//	{
//		//update_yaw(yaw);
//	}
//		Update_yaw(aaaa);
//}






