#include "include.h"
#include "data_pack.h"
uint8_t task_step = 0xff;
uint8_t if_zhaokaung_task_finish = 0;
uint8_t if_jiaozhun_task_finish = 0;

extern uint8_t cmd_myfromPc;
extern uint8_t rx_datafromPc[10];
extern uint8_t cmd_myfromChassis;
extern uint8_t rx_datafromChassis[10];
uint8_t tx_data_buf[10];
int motor_speed[3] = {0, 0, 0};
extern osThreadId motor_taskHandle;
extern osThreadId track_ballHandle;
extern osThreadId scheduler_TaskHandle;
extern osThreadId zhaokaungtaskHandle;
extern osThreadId jiaozhuntaskHandle;
extern osThreadId grapballtaskHandle;

void motor_control_task(void const *argument)
{
	//	static uint8_t point_step[2] = {0, 1}; //[0]  上次step    [1]now_step

	// 空闲一段时间

	// uint8_t flage1 = 40;
	for (;;)
	{
		CAN1_CMD_1(PID_velocity_realize_chassis_3508(motor_speed[0], 1), PID_velocity_realize_chassis_3508(-motor_speed[1], 2), PID_velocity_realize_chassis_3508(-motor_speed[2], 3), 0);
		osDelay(12);
	}
}
uint8_t if_stop_target = 0;
uint8_t if_move = 0;
int16_t dx = 0;
int16_t distance_carmere = 0;
int16_t buf[2];
extern pid_type_def pid_track_ball_x;
void track_ball_task(void const *argument)
{
	for (;;)
	{
		if (if_stop_target)
		{
			buf[0] = 0;
			buf[1] = 0;
			motor_speed[2] = 0;
			motor_speed[1] = 0;
			motor_speed[0] = 0;
			memcpy(&tx_data_buf[0], &buf[0], 2);
			memcpy(&tx_data_buf[2], &buf[1], 2);
			for (int i = 0; i < 10; i++)
			{
				Send_Cmd_Data2chassis(0x28, tx_data_buf, 4);
			}
			vTaskSuspend(NULL);
		}
		else
		{
			// int xishu[2] = {1, 3};
			int xishu[2] = {2, 2};
			//		buf[1] = PID_chassis_track_ball_x(dx);
			//		buf[0] = -PID_chassis_track_ball_distance(distance_carmere);

			//		if (distance_carmere < 1000)
			//		{
			//			//motor_speed[2] = 2000;
			//		}
			// if (distance_carmere < 0)
			// {
			// 	distance_carmere = 0;
			// }
			// else if (distance_carmere > 2000)
			// {
			// 	xishu[0] = 1;
			// }
			// else if (distance_carmere > 1000)
			// {
			// 	xishu[0] = 2;
			// }
			// else if (distance_carmere > 500)
			// {
			// 	xishu[0] = 3;
			// }
			// else if (distance_carmere < 240)
			// {

			// 	//osdelay(600);
			// }

			// else
			// {
			// 	xishu[0] = 3;
			// }

			// if (dx > 2000)
			// {
			// 	xishu[1] = 1;
			// }
			// else if (dx > 1000)
			// {
			// 	xishu[1] = 2;
			// }
			// else if (dx > 200)
			// {
			// 	xishu[1] = 3;
			// }
			// else
			// {
			// 	xishu[1] = 4;
			// }

			// buf[1] = -xishu[1] * dx;
			// buf[0] = xishu[0] * (distance_carmere + 100);

			buf[1] = xishu[1] * dx;
			buf[0] = xishu[0] * distance_carmere;
			if (buf[0] < 0)
			{
				buf[0] = 0;
			}

			memcpy(&tx_data_buf[0], &buf[0], 2);
			memcpy(&tx_data_buf[2], &buf[1], 2);
			Send_Cmd_Data2chassis(0x28, tx_data_buf, 4);
		}

		osDelay(20);

		// vTaskSuspend(NULL);
	}
}
uint8_t times = 0;
void scheduler_task(void const *argument)
{

	for (;;)
	{
		uint8_t buf = 0;
		if ((task_step == 0) || (task_step == 0xff))
		{

			if (task_step == 0xff) // 第一次到三区
			{

				buf = 0x06;
				SendPakageWaitACK(Chassis, 0x48, &buf, 1, 0xff); // 向底盘发送<0x48,0x06>信息  校准位置命令 并且接受底盘的应答

				buf = 0x00;
				WaitPakageSendACK(Chassis, 0x48, &buf, 1, 0xff); // 等待底盘完成<0x48,0x06>任务（位置校准）  并应答底盘发送的任务完成信息

				buf = 0x01;
				SendPakageWaitACK(Pc, 0x01, &buf, 1, 0x01); // 向pc发送<0x01,0x01>号信息
			}
			else //
			{
				buf = 0x06;
				SendPakageWaitACK(Chassis, 0x48, &buf, 1, 0xff); // 向底盘发送<0x48,0x06>信息  校准位置命令 并且接受底盘的应答

				buf = 0x00;
				WaitPakageSendACK(Chassis, 0x48, &buf, 1, 0xff); // 等待底盘完成<0x48,0x06>任务（位置校准）  并应答底盘发送的任务完成信息

				buf = 0x01;
				SendPakageWaitACK(Pc, 0x02, &buf, 1, 0x01); // 向pc发送<0x02,0x01>号信息
			}

			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
			task_step = 1;
		}
		else if (task_step == 1) // 找球
		{	         //************************************************************************追球 */

			vTaskResume(grapballtaskHandle);
			buf = 0x01;

			WaitPakageSendACK(Pc, 0x06, &buf, 1, 0x01); // 等待检测到红球进入框内

			motor_speed[2] = 0;
			osDelay(900); // 等待时间
			motor_speed[1] = 0;
			motor_speed[0] = 0;
			task_step = 2;
		}
		else if (task_step == 2)
		{
//码盘去中央位置找框
			buf=0x00;
			SendPakageWaitACK(Chassis, 0x51, &buf, 1, 0xff);//默认到能看到框的位置
			vTaskResume(zhaokaungtaskHandle);
			
			
			buf = 0x00;
			WaitPakageSendACK(Chassis, 0x50, &buf, 1, 0xff); // 等待底盘到达目标框

			vTaskSuspend(zhaokaungtaskHandle);

			// 放球
			motor_speed[1] = 9000;
			motor_speed[0] = 9000;
			osDelay(2000);
			motor_speed[1] = 0;
			motor_speed[0] = 0;
			// while (!((cmd_myfromPc == 0X02) && (rx_datafromPc[0] == 0x01))) // 找框任务完成  发送结束信息给工控
			// {
			// 	buf = 0x01;
			// 	Send_Cmd_Data2pc(0x02, &buf, 1);
			// 	osDelay(5);
			// }

			task_step = 0;
		}
		else
		{
			osDelay(100);
		}
		osDelay(10);
	}
}

void grapball_task(void const *argument) // 找球任务
{
	for (;;)
	{
		uint8_t tx_buff1 = 0x01;

		if ((cmd_myfromPc == 0x08) && (rx_datafromPc[0] == 0x88)) // 丢失目标
		{

			while ((cmd_myfromPc == 0x08) && (rx_datafromPc[0] == 0x88))
			{
				cmd_myfromPc = 0xff;
				Send_Cmd_Data2pc(0x08, &tx_buff1, 1);
				osDelay(26);
			}
			cmd_myfromPc = 0xff;
			if_stop_target = 1;
			osDelay(300);
			vTaskSuspend(track_ballHandle);
		}
		else if (cmd_myfromPc == 0x07) // 追球任务开始判断
		{

			motor_speed[2] = 4000;
			motor_speed[1] = 5000;
			motor_speed[0] = 5000;
			osDelay(500);
			vTaskResume(track_ballHandle);
			cmd_myfromPc = 0xff;
			osDelay(2);
		}
		else if ((cmd_myfromPc == 0x06) && (rx_datafromPc[0] == 0x00)) // 捡到紫色球
		{
			tx_buff1 = 0x01;
			while ((cmd_myfromPc == 0x06) && (rx_datafromPc[0] == 0x00))
			{
				cmd_myfromPc = 0xff;
				Send_Cmd_Data2pc(0x06, &tx_buff1, 1);
				osDelay(20);
			}

			// cmd_myfromPc = 0xff;
		}

		osDelay(6);
	}
}

void zhaokaung_task(void const *argument)
{
	uint8_t tx_buff = 0;
	static uint8_t target_kuang[2] = {0, 0}; //[0] 最优解  ；[1]次优解

	for (;;)
	{

		if (task_step == 2) // 执行当前动作
		{
			if (cmd_myfromPc == 0x04) // 判断去几号框// 底盘任务直接到几号
			{
				// wenti
				tx_buff = 0x01;
				target_kuang[0] = rx_datafromPc[0];
				target_kuang[1] = rx_datafromPc[1];
				while (cmd_myfromPc == 0x04)
				{
					cmd_myfromPc = 0xff;
					Send_Cmd_Data2pc(0x04, &tx_buff, 1);
					osDelay(5);
				}

				SendPakageWaitACK(Chassis, 0x50, target_kuang, 1, 0xff); // 向底盘发送命令 到<target_kuang>号框

				// while ((!(cmd_myfromChassis == 0x48) && (rx_datafromChassis[0] == 0xff))) // 等待底盘发送接受消息应答
				// {
				// 	Send_Cmd_Data2chassis(0x48, &target_kuang, 1); //    带位置校准
				// 	osDelay(5);
				// }
				// cmd_myfromPc = 0xff;
			}
			else if (cmd_myfromPc == 0x05) // 到框
			{

				tx_buff = 0x01;

				while (cmd_myfromPc == 0x05) // 给pc端应答  收到了0x05 消息
				{
					cmd_myfromPc = 0xff;
					Send_Cmd_Data2pc(0x05, &tx_buff, 1);
					osDelay(10);
				}

				tx_buff = 0x00;
				SendPakageWaitACK(Chassis, 0x51, &tx_buff, 1, 0xff); // 向底盘发送命令 到能看到框的位置

				// while ((!(cmd_myfromChassis == 0x48) && (rx_datafromChassis[0] == 0xff))) // 等待底盘发送接受消息应答
				// {
				// 	Send_Cmd_Data2chassis(0x48, &tx_buff, 1); // 向底盘发送命令 到能看到框的位置
				// 	osDelay(5);
				// }
				// cmd_myfromPc = 0xff;
			}
			else if (cmd_myfromPc == 0x09)
			{
				tx_buff = 0x01;
				target_kuang[0] = rx_datafromPc[0];

				while (cmd_myfromPc == 0x09)
				{
					cmd_myfromPc = 0xff;
					Send_Cmd_Data2pc(0x09, &tx_buff, 1);
					osDelay(5);
				}

				SendPakageWaitACK(Chassis, 0x50, target_kuang, 1, 0xff); // 向底盘发送命令 到<target_kuang>号框
				cmd_myfromPc = 0xff;
			}
		}
		else
		{
			vTaskSuspend(NULL);
		}

		osDelay(3);
	}
}
void jiaozhun_task(void const *argument)
{

	while (1)
	{
		osDelay(30);
	}
}

//void zhaokaung_task(void const *argument)
//{
//	uint8_t tx_buff = 0;
//	static uint8_t target_kuang = 0;

//	for (;;)
//	{

//		if (task_step == 2) // 执行当前动作
//		{
//			if (cmd_myfromPc == 0x04) // 判断去几号框// 底盘任务直接到几号
//			{
//				// wenti
//				tx_buff = 0x01;
//				target_kuang = rx_datafromPc[0];
//				while (cmd_myfromPc == 0x04)
//				{
//					cmd_myfromPc = 0xff;
//					Send_Cmd_Data2pc(0x04, &tx_buff, 1);
//					osDelay(5);
//				}

//				SendPakageWaitACK(Chassis, 0x48, &target_kuang, 1, 0xff); // 向底盘发送命令 到<target_kuang>号框

//				// while ((!(cmd_myfromChassis == 0x48) && (rx_datafromChassis[0] == 0xff))) // 等待底盘发送接受消息应答
//				// {
//				// 	Send_Cmd_Data2chassis(0x48, &target_kuang, 1); //    带位置校准
//				// 	osDelay(5);
//				// }
//				// cmd_myfromPc = 0xff;
//			}
//			else if (cmd_myfromPc == 0x05)
//			{

//				tx_buff = 0x01;

//				while (cmd_myfromPc == 0x05) // 给pc端应答  收到了0x05 消息
//				{
//					cmd_myfromPc = 0xff;
//					Send_Cmd_Data2pc(0x05, &tx_buff, 1);
//					osDelay(10);
//				}

//				tx_buff = 0x05;
//				SendPakageWaitACK(Chassis, 0x48, &tx_buff, 1, 0xff); // 向底盘发送命令 到能看到框的位置

//				// while ((!(cmd_myfromChassis == 0x48) && (rx_datafromChassis[0] == 0xff))) // 等待底盘发送接受消息应答
//				// {
//				// 	Send_Cmd_Data2chassis(0x48, &tx_buff, 1); // 向底盘发送命令 到能看到框的位置
//				// 	osDelay(5);
//				// }
//				// cmd_myfromPc = 0xff;
//			}
//			else
//			{
//			}
//		}
//		else
//		{
//			vTaskSuspend(NULL);
//		}

//		osDelay(3);
//	}
//}