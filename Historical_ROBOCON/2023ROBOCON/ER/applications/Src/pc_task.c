#include "pc_task.h"
#include <stdio.h>
uint8_t bluetooth_rx[1];
extern fp32 speed_target_final[4];

extern fp32 motor_speed_3508_pid[3];//底盘3508参数
extern fp32 motor_yaw_pid[3];
extern pid_type_def pid_yaw;
extern pid_type_def pid_v_1[8];
extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];
extern osSemaphoreId PC_BinarySemHandle;


void pc_task(void const * argument)
{
	while(1)
	{
		vTaskSuspend(NULL);
		xSemaphoreTake(PC_BinarySemHandle,portMAX_DELAY);
//		taskDISABLE_INTERRUPTS();
//		printf("%f,%d,%d,%d,%d\r\n",speed_target_final[0],motor_can1[0].speed_rpm,motor_can1[1].speed_rpm,motor_can1[2].speed_rpm,motor_can1[3].speed_rpm);
//		taskENABLE_INTERRUPTS();
		osDelay(10);
	}
}

void Tuning_PID(uint8_t *pc_rx)
{
	sscanf((const char*)pc_rx,"p1=%f,i1=%f,d1=%f,p2=%f,i2=%f,d2=%f",&motor_speed_3508_pid[0],&motor_speed_3508_pid[1],&motor_speed_3508_pid[2],&motor_yaw_pid[0],&motor_yaw_pid[1],&motor_yaw_pid[2]);
	for(int i = 0;i < 4;i++)
	{
		pid_v_1[i].Kp = motor_speed_3508_pid[0];
		pid_v_1[i].Ki = motor_speed_3508_pid[1];
		pid_v_1[i].Kd = motor_speed_3508_pid[2];
	}
	pid_yaw.Kp = motor_yaw_pid[0];
	pid_yaw.Ki = motor_yaw_pid[1];
	pid_yaw.Kd = motor_yaw_pid[2];
}


///******************************************************************************************/
///* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

//#if 1
//#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
//__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
//__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

//#else
///* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
//#pragma import(__use_no_semihosting)

//struct __FILE
//{
//    int handle;
//    /* Whatever you require here. If the only file you are using is */
//    /* standard output using printf() for debugging, no file handling */
//    /* is required. */
//};

//#endif

///* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
//int _ttywrch(int ch)
//{
//    ch = ch;
//    return ch;
//}

///* 定义_sys_exit()以避免使用半主机模式 */
//void _sys_exit(int x)
//{
//    x = x;
//}

//char *_sys_command_string(char *cmd, int len)
//{
//    return NULL;
//}

///* FILE 在 stdio.h里面定义. */
//FILE __stdout;

///* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
//int fputc(int ch, FILE *f)
//{
//    while ((UART5->SR & 0X40) == 0);             /* 等待上一个字符发送完成 */

//    UART5->DR = (uint8_t)ch;                     /* 将要发送的字符 ch 写入到DR寄存器 */
//    return ch;
//}
//#endif
///***********************************************END*******************************************/


