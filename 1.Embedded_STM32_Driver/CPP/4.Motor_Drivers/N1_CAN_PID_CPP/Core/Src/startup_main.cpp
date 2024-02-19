#include "startup_main.h"
#include "bsp_delay.h"
#include "can_receive.h"
#include "pid_user.h"

void startup_main(void)
{
//�δ�ʱ����ʼ��
	bsp_delay.f4.Init(168);
	
//PID��ʼ��
	pid_controller.All_Device_Init();
	
//CANͨ���˲���ʼ��
	can_bus.bsp.Filter_Init(&hcan1);
	can_bus.bsp.Filter_Init(&hcan2);
	
//CANͨ�ſ���
	can_bus.bsp.CAN_Start(&hcan1);
	can_bus.bsp.CAN_Start(&hcan2);


	
	
#if isRTOS==0    	//������������
	for(;;)  //��ͬ��while(true)
	{
		
		//���γ�������ڶ�ʱ���ж�����п��Ƶ�����������������뿴��ʱ���жϻص�������ġ�
		//�·�д���ǵ�ͬ�ڶ�ʱ���ж��е�����ʵ�֡�
		
		
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
		
		//����ά����1-10ms�ڣ��ر��ǿ��ƽǶȵ�ʱ�򣬼���Ƶ�ʲ��˹���
		bsp_delay.f4.ms(1);
	}
#endif
}