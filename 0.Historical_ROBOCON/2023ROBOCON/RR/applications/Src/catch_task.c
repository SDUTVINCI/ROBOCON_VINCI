#include "catch_task.h"

fp32 Catch_Angle_Target[2];
fp32 Catch_Current_Target[2];
fp32 Catch_Ready_Angle_Target[2];

fp32 Push_Angle_Target[1];
fp32 Push_Current_Target[1];

int16_t push_cnt = 0;
int16_t push_cnt_pre = 0;
int16_t ch0_pre;

fp32 teeth0 = 17.00f;
fp32 teeth1 = 2.80f;
//fp32 teeth = 5.13f;
fp32 teeth = 4.65f;
fp32 teeth2 = 3.45f;
fp32 teeth3 = 4.65f;

extern osThreadId PNEUMATICHandle;

extern osMessageQId Catch_Target_QueueHandle;
extern osMessageQId Catch_Pre_QueueHandle;
extern osMessageQId Push_Go_QueueHandle;
extern osMessageQId Push_Origin_QueueHandle;
extern osSemaphoreId CP_Link_BinarySemHandle;

extern osThreadId CATCH_CURRENTHandle;

void catch_task(void const *argument)
{

	while (1)
	{
		if (rc_ctrl.rc.ch[0] < 400 && ch0_pre > 400 && ABS(rc_ctrl.rc.ch[1]) < 200)
		{
			if (push_cnt < 14)
			{
				push_cnt++;
				xSemaphoreGive(CP_Link_BinarySemHandle);
			}
		}
		if (rc_ctrl.rc.ch[0] > -400 && ch0_pre < -400 && ABS(rc_ctrl.rc.ch[1]) < 200)
		{
			if (push_cnt > 0)
			{
				push_cnt = 0;
			}
		}
		ch0_pre = rc_ctrl.rc.ch[0];
		if (push_cnt != push_cnt_pre)
		{
			if (push_cnt != 0)
			{
				if (push_cnt == 1)
				{
					Catch_Ready_Angle_Target[0] = -8192.0f * 36.0f * teeth0 / 37.0f * push_cnt;
					Catch_Ready_Angle_Target[1] = 8192.0f * 36.0f * teeth0 / 37.0f * push_cnt;

					Catch_Angle_Target[0] = Catch_Ready_Angle_Target[0];
					Catch_Angle_Target[1] = Catch_Ready_Angle_Target[1];
				}
				else
				{
					Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth / 37.0f * push_cnt;
					Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth / 37.0f * push_cnt;
					Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
					Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
					if(push_cnt == 2)
					{
					Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth1 / 37.0f * push_cnt;
					Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth1 / 37.0f * push_cnt;
					Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
					Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
					}
					if (push_cnt > 2 && push_cnt < 5)
					{
						Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth2 / 37.0f * push_cnt;
						Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth2 / 37.0f * push_cnt;
						Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
						Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
					}
					if(push_cnt >= 5)
					{
						Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth3 / 37.0f * (push_cnt-1);
						Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth3 / 37.0f * (push_cnt-1);
						Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
						Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
					}
//					if (push_cnt == 11)
//					{
//						Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth3 / 37.0f * push_cnt;
//						Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth3 / 37.0f * push_cnt;
//						Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
//						Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
//					}
				}
				xQueueSendToFront(Catch_Target_QueueHandle, Catch_Angle_Target, 1);
			}

			osDelay(800);

			if (push_cnt != 0)
			{
				Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth / 37.0f * (push_cnt - 1);
				Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth / 37.0f * (push_cnt - 1);
				Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
				Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
				if (push_cnt == 1)
				{
					//				Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth0 / 37.0f * push_cnt;
					//				Catch_Angle_Target[1] =  8192.0f * 36.0f * teeth0 / 37.0f * push_cnt;
					Catch_Angle_Target[0] = Catch_Ready_Angle_Target[0];
					Catch_Angle_Target[1] = Catch_Ready_Angle_Target[1];
				}
					if(push_cnt == 2)
					{
					Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth1 / 37.0f * (push_cnt-1);
					Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth1 / 37.0f * (push_cnt-1);
					Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
					Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
					}
				if (push_cnt > 2 && push_cnt < 5)
				{
					Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth2 / 37.0f * (push_cnt - 1);
					Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth2 / 37.0f * (push_cnt - 1);
					Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
					Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
				}
					if(push_cnt >= 5)
					{
						Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth3 / 37.0f * (push_cnt-2);
						Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth3 / 37.0f * (push_cnt-2);
						Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
						Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
					}
//				if (push_cnt == 11)
//				{
//					Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth3 / 37.0f * (push_cnt - 1);
//					Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth3 / 37.0f * (push_cnt - 1);
//					Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
//					Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
//				}

				xQueueSendToFront(Catch_Pre_QueueHandle, Catch_Angle_Target, 1);
			}

			osDelay(1200);
			if (push_cnt == 1)
			{
				Catch_Angle_Target[0] = Catch_Ready_Angle_Target[0];
				Catch_Angle_Target[1] = Catch_Ready_Angle_Target[1];
			}
			else
			{
				Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth / 37.0f * push_cnt;
				Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth / 37.0f * push_cnt;
				Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
				Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
					if(push_cnt == 2)
					{
					Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth1 / 37.0f * push_cnt;
					Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth1 / 37.0f * push_cnt;
					Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
					Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
					}
				if (push_cnt > 2 && push_cnt < 5)
				{
					Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth2 / 37.0f * push_cnt;
					Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth2 / 37.0f * push_cnt;
					Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
					Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
				}
					if(push_cnt >= 5)
					{
						Catch_Angle_Target[0] = -8192.0f * 36.0f * teeth3 / 37.0f * (push_cnt-1);
						Catch_Angle_Target[1] = 8192.0f * 36.0f * teeth3 / 37.0f * (push_cnt-1);
						Catch_Angle_Target[0] += Catch_Ready_Angle_Target[0];
						Catch_Angle_Target[1] += Catch_Ready_Angle_Target[1];
					}
				if (push_cnt == 0)
				{
					Catch_Angle_Target[0] = -8192.0f * 36.0f * 0.0f / 37.0f * push_cnt;
					Catch_Angle_Target[1] = 8192.0f * 36.0f * 0.0f / 37.0f * push_cnt;
				}
				xQueueSendToFront(Catch_Target_QueueHandle, Catch_Angle_Target, 1);
			}
		}
		push_cnt_pre = push_cnt;
	}
}

fp32 push_deg_angle;
void push_task(void const *argument)
{
	while (1)
	{
		xSemaphoreTake(CP_Link_BinarySemHandle, osWaitForever);
		osDelay(680);
		push_deg_angle = 70.0f;
		*Push_Angle_Target = -8192.0f * 36.0f * push_deg_angle / 360.0f;
		xQueueSendToFront(Push_Go_QueueHandle, Push_Angle_Target, 1);

		osDelay(1000);

		push_deg_angle = 0.8f;
		*Push_Angle_Target = 8192.0f * 36.0f * push_deg_angle / 360.0f;
		xQueueSendToFront(Push_Origin_QueueHandle, Push_Angle_Target, 1);

//		vTaskResume(PNEUMATICHandle);
	}
}

fp32 Queue_Catch_Angle_Target[2];
fp32 Queue_Push_Angle_Target[1];
fp32 CMD_Catch_Angle_Target[2];
fp32 CMD_Push_Angle_Target[1];
fp32 CMD_Catch_Current_Target[2];
fp32 CMD_Push_Current_Target[1];

void can2_cmd_task(void const *argument)
{
	static BaseType_t catch_target_status = pdFALSE;
	static BaseType_t catch_pre_status = pdFALSE;
	static BaseType_t push_go_status = pdFALSE;
	static BaseType_t push_origin_status = pdFALSE;

	TickType_t PreviousWakeTime = osKernelSysTick();
	while (1)
	{

		catch_target_status = xQueueReceive(Catch_Target_QueueHandle, Queue_Catch_Angle_Target, 1);
		if (catch_target_status == pdTRUE)
		{
			CMD_Catch_Angle_Target[0] = -*Queue_Catch_Angle_Target;
			CMD_Catch_Angle_Target[1] = *Queue_Catch_Angle_Target;
			catch_target_status = pdFALSE;
		}

		catch_pre_status = xQueueReceive(Catch_Pre_QueueHandle, Queue_Catch_Angle_Target, 1);
		if (catch_pre_status == pdTRUE)
		{
			CMD_Catch_Angle_Target[0] = -*Queue_Catch_Angle_Target;
			CMD_Catch_Angle_Target[1] = *Queue_Catch_Angle_Target;
			catch_pre_status = pdFALSE;
		}

		for (int i = 0; i < 2; i++)
		{
			CMD_Catch_Current_Target[i] = PID_call_2(CMD_Catch_Angle_Target[i], i + 1);
		}

		push_go_status = xQueueReceive(Push_Go_QueueHandle, Queue_Push_Angle_Target, 1);
		if (push_go_status == pdTRUE)
		{
			*CMD_Push_Angle_Target = *Queue_Push_Angle_Target;
			push_go_status = pdFALSE;
		}
		push_origin_status = xQueueReceive(Push_Origin_QueueHandle, Queue_Push_Angle_Target, 1);
		if (push_origin_status == pdTRUE)
		{
			*CMD_Push_Angle_Target = *Queue_Push_Angle_Target;
			push_origin_status = pdFALSE;
		}
		CMD_Push_Current_Target[0] = PID_call_2(CMD_Push_Angle_Target[0], 3);

		CAN2_CMD_1(CMD_Catch_Current_Target[0], CMD_Catch_Current_Target[1], CMD_Push_Current_Target[0], 0);
	}
}

fp32 servo_motor_angle_calc(fp32 set_angle)
{
	static fp32 pulse_per_deg;
	fp32 pulse;
	if (-90.0f <= set_angle && set_angle <= 90.0f)
	{
		pulse_per_deg = 1000.0f / 90.0f;
		pulse = 1500.0f + pulse_per_deg * set_angle;
	}
	else
	{
		pulse = 1500.0f;
	}
	return pulse;
}
