#include "include.h"
uint8_t cmd_speed_buf[1];
uint16_t yaogan;
uint8_t getball_cmd_buf[1] = {0x1f};
uint8_t saveball_cmd_buf[1] = {0xf1};
uint8_t nrf_key[20];
extern UART_HandleTypeDef huart3;
// extern pid_type_def pid_v_chassis_3508[3], pid_v_chassis_6020[3], pid_p_chassis_6020[3];
int target_gimbal = 0;
int target_grab_3508 = -70000;
int target_grab_2006 = 0;
int left_qv_miao = 0;
int right_qv_miao = 0;
int left_qv_miao_2006[3] = {100000, 75000, 85000};
int right_qv_miao_2006[3] = {100000, 75000, 85000};
extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];
uint8_t GrabActionTask_flag = 0;
extern osThreadId grab_taskHandle;
extern osThreadId gimbal_taskHandle;
extern osThreadId shoot_taskHandle;
extern uint8_t cmd_myfromChassis;
extern uint8_t rx_datafromChassis[10];
void GrabActionTask(void const *argument)
{

	for (;;)
	{
		vTaskSuspend(gimbal_taskHandle);
		target_gimbal = 0;
		target_grab_2006 = 8000;
		osDelay(1000);
		target_grab_3508 = -40000;
		Send_Cmd_Data2Gimbal(0x5d, getball_cmd_buf, 1); // 取球动作

		while (fabs(motor_can2[4].total_angle) >= 300)
		{
			osDelay(2);
		}
		osDelay(1000);

		target_grab_3508 = -70000;

		osDelay(2000);

		target_grab_2006 = -27500;
		osDelay(600);
		Send_Cmd_Data2Gimbal(0x5d, saveball_cmd_buf, 1); // 取球动作

		osDelay(600);
		target_grab_3508 = -300;
		vTaskResume(gimbal_taskHandle);
		vTaskSuspend(grab_taskHandle);
	}
}
void GimbalPostionTask(void const *argument)
{
	for (;;)
	{

		target_gimbal = (yaogan - 500) * 800;

		osDelay(3);
	}
}

void ShootTask(void const *argument)
{

	for (;;)
	{
		cmd_speed_buf[0] = fabs(yaogan - 500) * 0.16;
		Send_Cmd_Data2Gimbal(0xd5, cmd_speed_buf, 1);
		osDelay(6);
		vTaskSuspend(shoot_taskHandle);
	}
}

void PidCaclaTask(void const *argument)
{
	// 空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME); // 上电初始化  马盘初始化等待时间

	for (;;)
	{

		

				//CAN1_CMD_1(PID_call_2006(left_qv_miao_2006[0], 1), PID_call_2006(left_qv_miao_2006[1], 2), PID_call_2006(left_qv_miao_2006[2], 3), PID_call_3508(left_qv_miao, 4));

				//CAN1_CMD_2(PID_call_3508(target_grab_3508, 5), 0, 0, 0);

				//CAN2_CMD_1(PID_call_2_2006(right_qv_miao_2006[0], 1), PID_call_2_2006(right_qv_miao_2006[1], 2), PID_call_2_2006(right_qv_miao_2006[2], 3), PID_call_2_3508(right_qv_miao, 4));

				//CAN2_CMD_2(PID_call_2_2006(target_gimbal, 5), PID_call_2_2006(target_grab_2006, 6), 0, 0);
		osDelay(6);
	}
}

void LedTask(void const *argument)
{

	// 空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME); // 上电初始化  马盘初始化等待时间

	for (;;)
	{
		if (cmd_myfromChassis == 0x7f)
		{
//			for (int i = 0; i < 8; i++)
//			{
//				nrf_key[i] = (rx_datafromChassis[0] & (1 << i)) >> i;
//			}
//			for (int i = 0; i < 8; i++)
//			{
//				nrf_key[i + 8] = (rx_datafromChassis[1] & (1 << i)) >> i;
//			}
//			for (int i = 0; i < 4; i++)
//			{
//				nrf_key[i + 16] = (rx_datafromChassis[2] & (1 << (i + 4))) >> (i + 4);
//			}
//			yaogan = ((rx_datafromChassis[2] & 0x03) << 8) | rx_datafromChassis[3];
			
			
		}
		
		osDelay(20);
	}
}

void left_qvmiao_Task(void const *argument)
{
	for (;;)
	{
		static uint8_t step_left_qvmiao = 0;
		if (step_left_qvmiao == 0)
		{
			step_left_qvmiao++;
			for (int i = 0; i < 3; i++)
			{
				left_qv_miao_2006[i] = -11000.0;
			}
			osDelay(600);
			left_qv_miao = -340000;
			osDelay(600);
		}
		else if (step_left_qvmiao == 1)
		{
			step_left_qvmiao++;
			left_qv_miao = 0;
			osDelay(600);

			left_qv_miao_2006[0] = 30000;
			left_qv_miao_2006[2] = 30000;
			osDelay(600);
			left_qv_miao = -340000;
		}
		else if (step_left_qvmiao == 2)
		{
			step_left_qvmiao = 0;
			left_qv_miao = 0;
			osDelay(600);
			for (int i = 0; i < 3; i++)
			{
				left_qv_miao_2006[i] = 30000;
			}
		}
		else
		{
		}

		osDelay(600);

		vTaskSuspend(NULL);
	}
}

void right_qvmiao_Task(void const *argument)
{
	for (;;)
	{
		static uint8_t step_right_qvmiao = 0;
		if (step_right_qvmiao == 0)
		{
			step_right_qvmiao++;
			for (int i = 0; i < 3; i++)
			{
				right_qv_miao_2006[i] = -10000.0;
			}
			osDelay(600);
			right_qv_miao = 340000;
			osDelay(600);
		}
		else if (step_right_qvmiao == 1)
		{
			step_right_qvmiao++;
			right_qv_miao = 0;
			osDelay(600);
			right_qv_miao_2006[0] = 30000;
			right_qv_miao_2006[2] = 30000;
			osDelay(600);
			right_qv_miao = 340000;
		}
		else if (step_right_qvmiao == 2)
		{
			step_right_qvmiao = 0;
			right_qv_miao = 0;
			osDelay(600);
			for (int i = 0; i < 3; i++)
			{
				right_qv_miao_2006[i] = 30000.0;
			}
		}

		osDelay(600);

		vTaskSuspend(NULL);
	}
}