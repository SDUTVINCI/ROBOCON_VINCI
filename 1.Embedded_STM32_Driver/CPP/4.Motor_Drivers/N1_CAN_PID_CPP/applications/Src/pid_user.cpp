#include "pid_user.h"
#include "can_receive.h"


/*PID����������*/
PID_Controller pid_controller;
/***************/

//PID�������(�ṹ��)
pid_type_def pid_v_1[8],pid_pos_1[8];
pid_type_def pid_v_2[8],pid_pos_2[8];
pid_type_def pid_yaw;
pid_type_def pid_pos_x;
pid_type_def pid_pos_y;

//���PID����
fp32 motor_speed_3508_pid[3] = {15, 0, 1.2};//����3508����
fp32 motor_position_3508_pid[3] = {0.2, 0, 1};
fp32 motor_speed_2006_pid[3] = {10,0,0};//����2006����
fp32 motor_position_2006_pid[3] = {0.27,0.022,0.3};


//��λPID����
fp32 motor_yaw_pid[3] = {120,0,0.1};
fp32 motor_pos_x_pid[3] = {6,0,0};
fp32 motor_pos_y_pid[3] = {6,0,0};


/**
 * @brief       PID�豸��ʼ��
 * @param       void
 * @retval      void
 * @note        ���ｫ���е�PID�豸�Ĳ������г�ʼ��������Kp,Ki,Kd,I_limit(�����޷�),O_limit(���޷�)���������,����ֵ������pid_type_def����С�
 */
void PID_Controller::All_Device_Init(void)
{
	//����PID
	for(int i=0;i<4;i++)
	{
    this->core.PID_Init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		this->core.PID_Init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid, 400, 300);
		
		this->core.PID_Init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 9000, 6000);
		this->core.PID_Init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 8000, 2000);
	}
	//������λ���PID
	for(int i=4;i<8;i++)
	{		
    this->core.PID_Init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		this->core.PID_Init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid, 400, 300);
		
		this->core.PID_Init(&pid_v_2[i], PID_POSITION, motor_speed_2006_pid, 10000, 6000);
		this->core.PID_Init(&pid_pos_2[i], PID_POSITION, motor_position_2006_pid, 2000, 300);
	}
	
  //��λPID
	this->core.PID_Init(&pid_yaw,PID_POSITION,motor_yaw_pid,3000,1500);
	this->core.PID_Init(&pid_pos_x,PID_POSITION,motor_pos_x_pid,1500,1500);
	this->core.PID_Init(&pid_pos_y,PID_POSITION,motor_pos_y_pid,1500,1500);
}

/**
 * @brief       CAN1�ٶȻ�
 * @param       set_speed���ٶ�rpm
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::CAN_MOTOR::CAN1_Velocity_Realize(fp32 set_speed,int i)
{
	pid_controller.core.PID_Calc(&pid_v_1[i],can_bus.motor_can1[i].speed_rpm , set_speed);
	return pid_v_1[i].out;
}

/**
 * @brief       CAN1λ�û�
 * @param       set_pos���Ƕ�ֵ��Ϊ��ԽǶ�ֵ�����꿴��˵����
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::CAN_MOTOR::CAN1_Position_Realize(fp32 set_pos,int i)
{
	pid_controller.core.PID_Calc(&pid_pos_1[i],can_bus.motor_can1[i].total_angle , set_pos);
	return pid_pos_1[i].out;
}

/**
 * @brief       CAN1�����ٶ�˫��
 * @param       set_pos���Ƕ�ֵ��Ϊ��ԽǶ�ֵ�����꿴��˵����
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::CAN_MOTOR::CAN1_VP_Dual_Loop_Realize(fp32 set_pos,int i)
{
	return CAN1_Velocity_Realize(CAN1_Position_Realize(set_pos,i),i);
}

/**
 * @brief       CAN2�ٶȻ�
 * @param       set_speed���ٶ�rpm
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::CAN_MOTOR::CAN2_Velocity_Realize(fp32 set_speed,int i)
{
	pid_controller.core.PID_Calc(&pid_v_2[i],can_bus.motor_can2[i].speed_rpm , set_speed);
	return pid_v_2[i].out;
}

/**
 * @brief       CAN2λ�û�
 * @param       set_pos���Ƕ�ֵ��Ϊ��ԽǶ�ֵ�����꿴��˵����
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::CAN_MOTOR::CAN2_Position_Realize(fp32 set_pos,int i)
{
	pid_controller.core.PID_Calc(&pid_pos_2[i],can_bus.motor_can2[i].total_angle , set_pos);
	return pid_pos_2[i].out;
}

/**
 * @brief       CAN2�����ٶ�˫��
 * @param       set_pos���Ƕ�ֵ��Ϊ��ԽǶ�ֵ�����꿴��˵����
 * @param       i��������Ϊ��ŵģ�Ҳ����i=���ID��-1
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::CAN_MOTOR::CAN2_VP_Dual_Loop_Realize(fp32 set_pos,int i)
{
	return CAN2_Velocity_Realize(CAN2_Position_Realize(set_pos,i),i);
}

/**
 * @brief       �����PID
 * @param       set_yaw��Ŀ�꺽���
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::SENSORS::Yaw_Realize(fp32 set_yaw)
{
	//PID_calc(&pid_yaw,absolute_chassis_measure.Euler.yaw_total,set_yaw);
	//return pid_yaw.out;
	(void)set_yaw;
	return 0;
}

/**
 * @brief       X����PID
 * @param       set_pos_x��Ŀ��X����ֵ
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::SENSORS::Pos_X_Realize(fp32 set_pos_x)
{
	//PID_calc(&pid_pos_x,absolute_chassis_measure.Position.Pos_X,set_pos_x);
	//return pid_pos_x.out;
	(void)set_pos_x;
	return 0;
}

/**
 * @brief       Y����PID
 * @param       set_pos_y��Ŀ��Y����ֵ
 * @retval      ���ֵ
 * @note        ���ֵ������ʲôֵ����Ҫ���ú��������ֵ��������ʲô��
 */
fp32 PID_Controller::SENSORS::Pos_Y_Realize(fp32 set_pos_y)
{
	//PID_calc(&pid_pos_y,absolute_chassis_measure.Position.Pos_Y,set_pos_y);
	//return pid_pos_y.out;
	(void)set_pos_y;
	return 0;
}

