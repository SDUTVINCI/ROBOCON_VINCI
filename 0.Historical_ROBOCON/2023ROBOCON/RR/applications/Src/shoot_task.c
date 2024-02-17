#include "shoot_task.h"

fp32 Shoot_Speed_Target[2];

fp32 Shoot_Current_Target[2];

extern RC_ctrl_t rc_ctrl;

void shoot_task(void const * argument)
{
	
	while(1)
	{
		if(rc_ctrl.rc.s[0] == 3)
		{
		if (rc_ctrl.rc.s[1] == 1)   //��ť �������棬�������ٶ�Ϊ���٣���2������
		{
			Shoot_Speed_Target[0] =  -3500;
			Shoot_Speed_Target[1] =  -Shoot_Speed_Target[0];
		}
		else if(rc_ctrl.rc.s[1] == 2) //��ť �������棬�������ٶ�Ϊ���٣���1������
		{
			Shoot_Speed_Target[0] =  -3000;
			Shoot_Speed_Target[1] =  -Shoot_Speed_Target[0];
		}
		else if(rc_ctrl.rc.s[1] == 3) //��ť ���м䣬�������ٶ�Ϊ0����ֹ
		{
			Shoot_Speed_Target[0] =  0;
			Shoot_Speed_Target[1] =  -Shoot_Speed_Target[0];
		}
		}
		
		if(rc_ctrl.rc.s[0] == 1)
		{
		if (rc_ctrl.rc.s[1] == 1)   //��ť �������棬�������ٶ�Ϊ���٣���2������
		{
			Shoot_Speed_Target[0] =  -4500;
			Shoot_Speed_Target[1] =  -Shoot_Speed_Target[0];
		}
		else if(rc_ctrl.rc.s[1] == 2) //��ť �������棬�������ٶ�Ϊ���٣���1������
		{
			Shoot_Speed_Target[0] =  -4000;
			Shoot_Speed_Target[1] =  -Shoot_Speed_Target[0];
		}
		else if(rc_ctrl.rc.s[1] == 3) //��ť ���м䣬�������ٶ�Ϊ0����ֹ
		{
			Shoot_Speed_Target[0] =  0;
			Shoot_Speed_Target[1] =  0;
		}
		}
		
		Shoot_Current_Target[0] = PID_velocity_realize_1(Shoot_Speed_Target[0],5);
		Shoot_Current_Target[1] = PID_velocity_realize_1(Shoot_Speed_Target[1],6);
		
		CAN1_CMD_2(Shoot_Current_Target[0],Shoot_Current_Target[1],0,0);
		osDelay(2);
	}
}


