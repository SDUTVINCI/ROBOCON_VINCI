#include "timer_user.h"
#include "can_receive.h"
#include "pid_user.h"

extern "C"
/**
 * @brief       �ص���������ʱ���жϷ���������
 * @param       htim�������жϵĶ�ʱ�����
 * @retval      void
 * @note        �ú������жϹ������������ã������û�ȥ���á���Ϊһ����������������C++��Ҫ�ڸú���ǰ�����extern "C"����ֱ����extern "C"{}������
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM7)  //����Ϊ1ms
	{
		//������ʹCAN1��1�ŵ����1500����Ե���ֵ����ת��
		can_bus.cmd.CAN1_Front(1500,0,0,0);
		
		
		/*�ջ�    ��������0��ʼ�����ID��1��ʼ��
		ʹCAN1�������Ϊ4(ʵ����IDΪ5)�ĵ����500rpm���ٶ�ת����
		ʹCAN1�������Ϊ5(ʵ����IDΪ6)�ĵ����600rpm���ٶ�ת����
		ʹCAN1�������Ϊ6(ʵ����IDΪ7)�ĵ��ת����4000����ԽǶ�ͣ�¡�
		ʹCAN1�������Ϊ7(ʵ����IDΪ8)�ĵ��������0��Ե���ֵ������
		*/
		can_bus.cmd.CAN1_Behind(
		pid_controller.can_motor.CAN1_Velocity_Realize(500,4),
		pid_controller.can_motor.CAN1_Velocity_Realize(600,5),
		pid_controller.can_motor.CAN1_Position_Realize(4000,6),
		0);
	}
}
