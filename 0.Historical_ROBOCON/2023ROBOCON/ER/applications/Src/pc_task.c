#include "pc_task.h"
#include <stdio.h>
uint8_t bluetooth_rx[1];
extern fp32 speed_target_final[4];

extern fp32 motor_speed_3508_pid[3];//����3508����
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
///* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

//#if 1
//#if (__ARMCC_VERSION >= 6010050)                    /* ʹ��AC6������ʱ */
//__asm(".global __use_no_semihosting\n\t");          /* ������ʹ�ð�����ģʽ */
//__asm(".global __ARM_use_no_argv \n\t");            /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

//#else
///* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
//#pragma import(__use_no_semihosting)

//struct __FILE
//{
//    int handle;
//    /* Whatever you require here. If the only file you are using is */
//    /* standard output using printf() for debugging, no file handling */
//    /* is required. */
//};

//#endif

///* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
//int _ttywrch(int ch)
//{
//    ch = ch;
//    return ch;
//}

///* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
//void _sys_exit(int x)
//{
//    x = x;
//}

//char *_sys_command_string(char *cmd, int len)
//{
//    return NULL;
//}

///* FILE �� stdio.h���涨��. */
//FILE __stdout;

///* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
//int fputc(int ch, FILE *f)
//{
//    while ((UART5->SR & 0X40) == 0);             /* �ȴ���һ���ַ�������� */

//    UART5->DR = (uint8_t)ch;                     /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
//    return ch;
//}
//#endif
///***********************************************END*******************************************/


