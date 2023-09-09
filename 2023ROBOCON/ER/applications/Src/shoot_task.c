#include "shoot_task.h"
extern RC_ctrl_t rc_ctrl;
fp32 Shoot_Speed_Preset[2];
fp32 Shoot_Speed_Target[2];
fp32 Shoot_Current_Target[2];
int8_t rc_s_pre = 0;
fp32 accel_time = 0;
int16_t correct_ch1_pre;
extern osThreadId PNEUMATICHandle;
extern osSemaphoreId CP_Link_BinarySemHandle;
extern uint8_t bluetooth_rx[1];
uint8_t bluetooth_rx_pre;
fp32 speed_delta[2] = {0,0};
fp32 shoot_speed_final[2];
fp32 speed_ratio = 1875.0f / 3075.0f;
//fp32 shoot_accel[2];
//fp32 shoot_accel_time[2];


void shoot_task(void const * argument)
{
	while(1)
	{
//		//靠河速度
//		if(rc_ctrl.rc.s[0] == 3)
//		{
//		if (rc_ctrl.rc.s[1] == 1 && rc_s_pre != 1)   //左按钮 置最上面，发射电机速度为高速，射2型立柱
//		{
//			Shoot_Speed_Preset[0] =  2800;
//			Shoot_Speed_Preset[1] = -2800;
//			accel_time = 0;
//		
//		}
//		else if(rc_ctrl.rc.s[1] == 2 && rc_s_pre != 2) //左按钮 置最下面，发射电机速度为低速，射1型立柱
//		{
//			Shoot_Speed_Preset[0] =  2650;
//			Shoot_Speed_Preset[1] = -2650;
//			accel_time = 0;
//		}
//		else if(rc_ctrl.rc.s[1] == 3 && rc_s_pre != 3) //左按钮 置中间，发射电机速度为0，静止
//		{
//			Shoot_Speed_Preset[0] =  0;
//			Shoot_Speed_Preset[1] =  0;
//			accel_time = 0;
//		}
//		}
//		
//		
//		if(rc_ctrl.rc.s[0] == 1)
//		{
//		if (rc_ctrl.rc.s[1] == 1 && rc_s_pre != 1)   //左按钮 置最上面，发射电机速度为高速，射2型立柱
//		{
//			Shoot_Speed_Preset[0] =  3520;
//			Shoot_Speed_Preset[1] = -3520;
//			accel_time = 0;
//		
//		}
//		else if(rc_ctrl.rc.s[1] == 2 && rc_s_pre != 2) //左按钮 置最下面，发射电机速度为低速，射1型立柱
//		{
//			Shoot_Speed_Preset[0] =  3755;
//			Shoot_Speed_Preset[1] = -3755;
//			accel_time = 0;
//		}
//		else if(rc_ctrl.rc.s[1] == 3 && rc_s_pre != 3) //左按钮 置中间，发射电机速度为0，静止
//		{
//			Shoot_Speed_Preset[0] =  0;
//			Shoot_Speed_Preset[1] =  0;
//			accel_time = 0;
//		}
//		}
		
		
		
////不靠河速度		
//		if(rc_ctrl.rc.s[0] == 3)
//		{
//		if (rc_ctrl.rc.s[1] == 1 && rc_s_pre != 1)   //左按钮 置最上面，发射电机速度为高速，射2型立柱
//		{
//			Shoot_Speed_Preset[0] =  2750;
//			Shoot_Speed_Preset[1] = -2750;
//			accel_time = 0;
//		
//		}
//		else if(rc_ctrl.rc.s[1] == 2 && rc_s_pre != 2) //左按钮 置最下面，发射电机速度为低速，射1型立柱
//		{
//			Shoot_Speed_Preset[0] =  2335;
//			Shoot_Speed_Preset[1] = -2335;
//			accel_time = 0;
//		}
//		else if(rc_ctrl.rc.s[1] == 3 && rc_s_pre != 3) //左按钮 置中间，发射电机速度为0，静止
//		{
//			Shoot_Speed_Preset[0] =  0;
//			Shoot_Speed_Preset[1] =  0;
//			accel_time = 0;
//		}
//		}
//		
//		
//		if(rc_ctrl.rc.s[0] == 1)
//		{
//		if (rc_ctrl.rc.s[1] == 1 && rc_s_pre != 1)   //左按钮 置最上面，发射电机速度为高速，射2型立柱
//		{
//			Shoot_Speed_Preset[0] =  3480;
//			Shoot_Speed_Preset[1] = -3480;
//			accel_time = 0;
//		
//		}
//		else if(rc_ctrl.rc.s[1] == 2 && rc_s_pre != 2) //左按钮 置最下面，发射电机速度为低速，射1型立柱
//		{
//			Shoot_Speed_Preset[0] =  3540;
//			Shoot_Speed_Preset[1] = -3540;
//			accel_time = 0;
//		}
//		else if(rc_ctrl.rc.s[1] == 3 && rc_s_pre != 3) //左按钮 置中间，发射电机速度为0，静止
//		{
//			Shoot_Speed_Preset[0] =  0;
//			Shoot_Speed_Preset[1] =  0;
//			accel_time = 0;
//		}
//		}



//24℃场馆
		if(rc_ctrl.rc.s[0] == 3)
		{
		if (rc_ctrl.rc.s[1] == 1 && rc_s_pre != 1)   //左按钮 置最上面，发射电机速度为高速，射2型立柱
		{
			Shoot_Speed_Preset[0] =  2815;
			Shoot_Speed_Preset[1] = -2815;
			accel_time = 0;
		
		}
		else if(rc_ctrl.rc.s[1] == 2 && rc_s_pre != 2) //左按钮 置最下面，发射电机速度为低速，射1型立柱
		{
//			Shoot_Speed_Preset[0] =  3075;//2400
//			Shoot_Speed_Preset[1] = -Shoot_Speed_Preset[0] * speed_ratio;
			Shoot_Speed_Preset[0] =  3075;
			Shoot_Speed_Preset[1] = -1875;
			accel_time = 0;
		}
		else if(rc_ctrl.rc.s[1] == 3 && rc_s_pre != 3) //左按钮 置中间，发射电机速度为0，静止
		{
			Shoot_Speed_Preset[0] =  0;
			Shoot_Speed_Preset[1] =  0;
			accel_time = 0;
		}
		}
		
		
		if(rc_ctrl.rc.s[0] == 1)
		{
		if (rc_ctrl.rc.s[1] == 1 && rc_s_pre != 1)   //左按钮 置最上面，发射电机速度为高速，射2型立柱
		{
			Shoot_Speed_Preset[0] =  3461.5;
			Shoot_Speed_Preset[1] = -3461.5;
//			Shoot_Speed_Preset[0] = 4000;
//			Shoot_Speed_Preset[1] = -Shoot_Speed_Preset[0] * speed_ratio;
			accel_time = 0;
		
		}
		else if(rc_ctrl.rc.s[1] == 2 && rc_s_pre != 2) //左按钮 置最下面，发射电机速度为低速，射1型立柱
		{
			Shoot_Speed_Preset[0] =  3600;  //三星柱3600
			Shoot_Speed_Preset[1] = -3600;
//			Shoot_Speed_Preset[0] = 4680;
//			Shoot_Speed_Preset[1] = -Shoot_Speed_Preset[0] * speed_ratio;
			accel_time = 0;
		}
		else if(rc_ctrl.rc.s[1] == 3 && rc_s_pre != 3) //左按钮 置中间，发射电机速度为0，静止
		{
			Shoot_Speed_Preset[0] =  0;
			Shoot_Speed_Preset[1] =  0;
			accel_time = 0;
		}
		}
		

		rc_s_pre = rc_ctrl.rc.s[1];
		
		
		/****************************************correct_task纠正任务分支********************************************************************/		
//		if(*bluetooth_rx == 0x00 && bluetooth_rx_pre != 0x00)
//		{
//			speed_delta[0] = 0.0f;
//			speed_delta[1] = -speed_delta[0];
//		}
//		
//		
//		if(*bluetooth_rx == 0x01 && bluetooth_rx_pre != 0x01)
//		{
//			speed_delta[0] += 1.0f;
//			speed_delta[1] = -speed_delta[0];
//		}
//		if(*bluetooth_rx == 0x02 && bluetooth_rx_pre != 0x02)
//		{
//			speed_delta[0] += 2.5f;
//			speed_delta[1] = -speed_delta[0];
//		}
//		if(*bluetooth_rx == 0x03 && bluetooth_rx_pre != 0x03)
//		{
//			speed_delta[0] += 5.0f;
//			speed_delta[1] = -speed_delta[0];
//		}
//		if(*bluetooth_rx == 0x04 && bluetooth_rx_pre != 0x04)
//		{
//			speed_delta[0] += 50.0f;
//			speed_delta[1] = -speed_delta[0];
//		}

//		
//		if(*bluetooth_rx == 0x05 && bluetooth_rx_pre != 0x05)
//		{
//			speed_delta[0] -= 1.0f;
//			speed_delta[1] = -speed_delta[0];
//		}
//		if(*bluetooth_rx == 0x06 && bluetooth_rx_pre != 0x06)
//		{
//			speed_delta[0] -= 2.5f;
//			speed_delta[1] = -speed_delta[0];
//		}
//		if(*bluetooth_rx == 0x07 && bluetooth_rx_pre != 0x07)
//		{
//			speed_delta[0] -= 5.0f;
//			speed_delta[1] = -speed_delta[0];
//		}
//		if(*bluetooth_rx == 0x08 && bluetooth_rx_pre != 0x08)
//		{
//			speed_delta[0] -= 50.0f;
//			speed_delta[1] = -speed_delta[0];
//		}
		
		
		for(int i = 0;i < 2;i++)
		{
			shoot_speed_final[i] = Shoot_Speed_Preset[i] + speed_delta[i];
		}
		
		
		//卡环纠正
		if(rc_ctrl.rc.s[0] == 3 || rc_ctrl.rc.s[0] == 1)
		{
			if(rc_ctrl.rc.ch[1] <= 460 && correct_ch1_pre >= 460 && ABS(rc_ctrl.rc.ch[0]) <=300)
			{
				xSemaphoreGive(CP_Link_BinarySemHandle);
			}
			if(rc_ctrl.rc.ch[1] >= -460 && correct_ch1_pre <= -460 && ABS(rc_ctrl.rc.ch[0]) <=300)
			{
				vTaskResume(PNEUMATICHandle);
			}
		}	
		
		correct_ch1_pre = rc_ctrl.rc.ch[1];
		bluetooth_rx_pre = *bluetooth_rx;

		
		for(int i = 0;i < 2;i++)
		{
			Shoot_Speed_Target[i] = accel_optimize(shoot_speed_final[i],1500);
			Shoot_Current_Target[i] = PID_velocity_realize_1(Shoot_Speed_Target[i],i+5);
		}
		CAN1_CMD_2(Shoot_Current_Target[0],Shoot_Current_Target[1],0,0);
		osDelay(2);
	}
}


//防止急停，线性加速度函数
fp32 accel_optimize(fp32 speed_target,fp32 interval_time)
{
	
	fp32 speed,shoot_accel;
	shoot_accel = speed_target / interval_time;
	speed = shoot_accel * accel_time;
	if(ABS(speed) <= ABS(speed_target))
	{
	 accel_time++;
	}
	return speed;
}

